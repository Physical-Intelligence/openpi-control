/*!
 * @file pi_driver_can.cpp
 * @brief Implementation of the DriverCan class for CAN bus communication.
 *
 * This package is Linux-only and uses SocketCAN (PF_CAN, SOCK_RAW,
 * CAN_RAW) for classic CAN frame handling.
 */

#include <sys/ioctl.h>
#include <unistd.h>
#include <cstring>

#include <net/if.h>
#include <sys/socket.h>

#include "pi_driver_can.hpp"

// ----------------------------------------------------------------------------
// DriverCan public API
// ----------------------------------------------------------------------------

DriverCan::DriverCan(Device* p_device, const CommandLineArgs& cla)
    : Driver(p_device, cla), port_name_(cla.control_port_name), sock_(-1), is_running_(false) {
    p_data_buf_ = &data_frame_;
    data_buf_size_ = sizeof(can_frame_t);
}

DriverCan::~DriverCan() { close(); }

ReturnCode DriverCan::open(int baud_rate) {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (!wait_for_reception_join_locked(lifecycle_lock)) {
        return ReturnCode::BUSY;
    }
    if (sock_ >= 0) {
        return ReturnCode::SUCCESS;
    }

    (void)baud_rate;

    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        PI_ERROR("Failed to create CAN socket: %s", strerror(errno));
        return ReturnCode::FAIL;
    }

    int send_buf_size = 1048576 * 2;
    setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &send_buf_size, sizeof(send_buf_size));

    struct ifreq ifr {};
    strncpy(ifr.ifr_name, port_name_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        PI_ERROR("Failed to get interface index for '%s': %s", port_name_.c_str(), strerror(errno));
        close_locked(lifecycle_lock);
        return ReturnCode::FAIL;
    }

    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, (struct sockaddr*)&addr_, sizeof(addr_)) < 0) {
        PI_ERROR("Failed to bind socket to CAN interface '%s': %s", port_name_.c_str(), strerror(errno));
        close_locked(lifecycle_lock);
        return ReturnCode::FAIL;
    }

    PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
            "Classic CAN configured on interface %s", port_name_.c_str());

    return ReturnCode::SUCCESS;
}

ReturnCode DriverCan::close() {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    return close_locked(lifecycle_lock);
}

ReturnCode DriverCan::close_locked(std::unique_lock<std::mutex>& lifecycle_lock) {
    stop_reception_locked(lifecycle_lock);

    if (sock_ >= 0) {
        ::close(sock_);
        sock_ = -1;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverCan::start_reception(const callback_t& callback) {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (!callback) {
        PI_ERROR("Cannot start reception: callback is empty");
        return ReturnCode::INVALID_PARAM;
    }
    while (true) {
        if (is_running_) {
            return ReturnCode::SUCCESS;
        }
        if (reception_thread_id_ == std::this_thread::get_id()) {
            return ReturnCode::BUSY;
        }
        if (reception_join_in_progress_) {
            lifecycle_cv_.wait(lifecycle_lock, [this] { return !reception_join_in_progress_; });
            continue;
        }
        if (!reception_thread_.joinable()) {
            break;
        }
        join_reception_thread_locked(lifecycle_lock);
    }
    if (sock_ < 0) {
        PI_ERROR("Cannot start reception: CAN socket is not initialized");
        return ReturnCode::FAIL;
    }

    is_running_ = true;
    reception_thread_ = std::thread(&DriverCan::receive_loop, this, callback);
    reception_thread_id_ = reception_thread_.get_id();

    return ReturnCode::SUCCESS;
}

ReturnCode DriverCan::stop_reception() {
    std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
    return stop_reception_locked(lifecycle_lock);
}

bool DriverCan::wait_for_reception_join_locked(std::unique_lock<std::mutex>& lifecycle_lock) {
    if (reception_join_in_progress_ && reception_thread_id_ == std::this_thread::get_id()) {
        return false;
    }
    lifecycle_cv_.wait(lifecycle_lock, [this] { return !reception_join_in_progress_; });
    return true;
}

void DriverCan::join_reception_thread_locked(std::unique_lock<std::mutex>& lifecycle_lock) {
    if (reception_join_in_progress_ || !reception_thread_.joinable()) {
        return;
    }
    if (reception_thread_id_ == std::this_thread::get_id()) {
        return;
    }

    reception_join_in_progress_ = true;
    std::thread worker = std::move(reception_thread_);
    lifecycle_lock.unlock();
    worker.join();
    lifecycle_lock.lock();

    reception_join_in_progress_ = false;
    reception_thread_id_ = std::thread::id{};
    lifecycle_cv_.notify_all();
}

ReturnCode DriverCan::stop_reception_locked(std::unique_lock<std::mutex>& lifecycle_lock) {
    while (true) {
        is_running_ = false;
        if (reception_thread_id_ == std::this_thread::get_id()) {
            return ReturnCode::SUCCESS;
        }
        if (reception_join_in_progress_) {
            lifecycle_cv_.wait(lifecycle_lock, [this] { return !reception_join_in_progress_; });
            continue;
        }
        join_reception_thread_locked(lifecycle_lock);
        return ReturnCode::SUCCESS;
    }
}

void DriverCan::receive_loop(const callback_t& callback) {
    if (sock_ < 0) {
        ReturnCode return_code = open(0);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to open CAN socket in receive_loop()");
            is_running_.store(false, std::memory_order_release);
            return;
        }
    }

    fd_set read_fds;
    timeval timeout;

    while (is_running_) {
        FD_ZERO(&read_fds);
        FD_SET(sock_, &read_fds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(sock_ + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            PI_ERROR("select() failed in receive_loop(): %s", strerror(errno));
            break;
        } else if (ret == 0) {
            continue;
        }

        if (FD_ISSET(sock_, &read_fds)) {
            ssize_t nbytes = read(sock_, p_data_buf_, data_buf_size_);
            if (nbytes < 0) {
                if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                PI_ERROR("Failed to read CAN frame: %s", strerror(errno));
                break;
            }
            callback(p_data_buf_, data_buf_size_, nbytes);
        }
    }
    is_running_.store(false, std::memory_order_release);
}

ReturnCode DriverCan::send_frame(void* p_data_buf, size_t data_buf_size) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (p_data_buf == nullptr || data_buf_size != sizeof(can_frame_t)) {
        PI_ERROR("Cannot send frame: invalid buffer (p_data_buf=%p, size=%zu)",
                 p_data_buf, data_buf_size);
        return ReturnCode::INVALID_PARAM;
    }
    if (sock_ < 0) {
        PI_ERROR("Cannot send frame: CAN socket is not initialized");
        return ReturnCode::FAIL;
    }

    // Prevent indefinite blocking on write when the interface is up but bus is off / no ACK.
    // Keep this small so higher-level code can fail fast.
    constexpr int kWriteTimeoutMs = 5;

    fd_set write_fds;
    FD_ZERO(&write_fds);
    FD_SET(sock_, &write_fds);

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = kWriteTimeoutMs * 1000L;

    int ret = select(sock_ + 1, nullptr, &write_fds, nullptr, &timeout);
    if (ret < 0) {
        if (errno == EINTR) {
            return ReturnCode::BUSY;
        }
        PI_ERROR("select() failed in send_frame(): %s", strerror(errno));
        return ReturnCode::FAIL;
    }
    if (ret == 0) {
        return ReturnCode::BUSY;
    }

    ssize_t nbytes = write(sock_, p_data_buf, data_buf_size);
    if (nbytes < 0) {
        PI_ERROR("Failed to send CAN frame: %s", strerror(errno));
        return ReturnCode::FAIL;
    }

    usleep(100);

    return ReturnCode::SUCCESS;
}

ReturnCode DriverCan::read_frame(void* p_data_buf, size_t data_buf_size) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (p_data_buf == nullptr || data_buf_size != sizeof(can_frame_t)) {
        PI_ERROR("Cannot read frame: invalid buffer (p_data_buf=%p, size=%zu)",
                 p_data_buf, data_buf_size);
        return ReturnCode::INVALID_PARAM;
    }
    if (sock_ < 0) {
        PI_ERROR("Cannot read frame: CAN socket is not initialized");
        return ReturnCode::FAIL;
    }

    // Prevent indefinite blocking when the bus is up but no frames arrive (e.g. servo disconnected).
    // Keep this small so higher-level code can fail fast.
    constexpr int kReadTimeoutMs = 5;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(sock_, &read_fds);

    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = kReadTimeoutMs * 1000L;

    int ret = select(sock_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ret < 0) {
        if (errno == EINTR) {
            return ReturnCode::NO_RESPONSE;
        }
        PI_ERROR("select() failed in read_frame(): %s", strerror(errno));
        return ReturnCode::FAIL;
    }
    if (ret == 0) {
        return ReturnCode::NO_RESPONSE;
    }

    iovec buffer{p_data_buf, data_buf_size};
    msghdr message{};
    message.msg_iov = &buffer;
    message.msg_iovlen = 1;
    ssize_t nbytes = recvmsg(sock_, &message, 0);
    if (nbytes < 0) {
        PI_ERROR("Failed to read CAN frame: %s", strerror(errno));
        return ReturnCode::FAIL;
    }
    if ((message.msg_flags & MSG_TRUNC) != 0) {
        PI_ERROR("CAN frame did not fit receive buffer: buffer=%zu, received=%zd",
                 data_buf_size, nbytes);
        return ReturnCode::FAIL;
    }
    if (static_cast<size_t>(nbytes) < sizeof(can_frame_t)) {
        PI_ERROR("Truncated CAN frame: read=%zd, expected at least=%zu", nbytes, sizeof(can_frame_t));
        return ReturnCode::FAIL;
    }

    return ReturnCode::SUCCESS;
}
