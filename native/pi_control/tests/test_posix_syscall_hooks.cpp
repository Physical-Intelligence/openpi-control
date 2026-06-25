#include "test_posix_syscall_hooks.hpp"

#if defined(__linux__)

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdarg>
#include <thread>

namespace test_posix_hooks {

std::atomic<bool> simulate_can_open{false};
std::atomic<int> simulated_can_socket{-1};

std::atomic<int> delayed_send_socket{-1};
std::atomic<bool> delay_send_after_select{false};
std::atomic<bool> send_select_returning{false};
std::atomic<bool> release_send_select{false};
std::atomic<bool> delayed_send_socket_closed{false};
std::atomic<bool> delayed_send_close_entered{false};
std::atomic<int> writes_after_send_socket_close{0};

std::atomic<int> delayed_read_socket{-1};
std::atomic<bool> delay_read_after_select{false};
std::atomic<bool> read_select_returning{false};
std::atomic<bool> release_read_select{false};
std::atomic<bool> delayed_read_socket_closed{false};
std::atomic<bool> delayed_read_close_entered{false};
std::atomic<int> reads_after_read_socket_close{0};

std::atomic<int> fail_next_read_select_fd{-1};
std::atomic<bool> fail_next_read_select{false};
std::atomic<bool> read_select_failure_injected{false};
std::atomic<bool> pause_read_select_after_failure{false};
std::atomic<bool> read_select_paused_after_failure{false};
std::atomic<bool> release_read_select_after_failure{false};

std::atomic<int> fail_next_read_fd{-1};
std::atomic<bool> fail_next_read{false};
std::atomic<bool> read_failure_injected{false};
std::atomic<bool> pause_read_after_failure{false};
std::atomic<bool> read_paused_after_failure{false};
std::atomic<bool> release_read_after_failure{false};

std::atomic<int> paused_read_fd{-1};
std::atomic<bool> pause_read_before_syscall{false};
std::atomic<bool> paused_read_entered{false};
std::atomic<bool> release_paused_read{false};

std::atomic<int> paused_write_fd{-1};
std::atomic<bool> capture_next_write_fd{false};
std::atomic<size_t> next_write_max_bytes{0};
std::atomic<bool> pause_write_before_syscall{false};
std::atomic<bool> paused_write_entered{false};
std::atomic<bool> release_paused_write{false};
std::atomic<bool> paused_write_fd_closed{false};
std::atomic<bool> paused_write_close_entered{false};
std::atomic<int> writes_after_paused_fd_close{0};

bool wait_for_flag(const std::atomic<bool>& flag, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (flag.load(std::memory_order_acquire)) {
            return true;
        }
        std::this_thread::yield();
    }
    return flag.load(std::memory_order_acquire);
}

}  // namespace test_posix_hooks

using namespace test_posix_hooks;

extern "C" {

int __real_socket(int domain, int type, int protocol);
int __wrap_socket(int domain, int type, int protocol) {
    if (simulate_can_open.load(std::memory_order_acquire) && domain == PF_CAN) {
        const int fd = __real_socket(AF_UNIX, SOCK_DGRAM, 0);
        simulated_can_socket.store(fd, std::memory_order_release);
        return fd;
    }
    return __real_socket(domain, type, protocol);
}

int __real_ioctl(int fd, unsigned long request, ...);
int __wrap_ioctl(int fd, unsigned long request, ...) {
    va_list args;
    va_start(args, request);
    void* arg = va_arg(args, void*);
    va_end(args);
    if (fd == simulated_can_socket.load(std::memory_order_acquire) && request == SIOCGIFINDEX) {
        static_cast<ifreq*>(arg)->ifr_ifindex = 1;
        return 0;
    }
    return __real_ioctl(fd, request, arg);
}

int __real_bind(int fd, const sockaddr* address, socklen_t address_length);
int __wrap_bind(int fd, const sockaddr* address, socklen_t address_length) {
    if (fd == simulated_can_socket.load(std::memory_order_acquire)) {
        return 0;
    }
    return __real_bind(fd, address, address_length);
}

int __real_setsockopt(int fd, int level, int option_name, const void* option_value,
                      socklen_t option_length);
int __wrap_setsockopt(int fd, int level, int option_name, const void* option_value,
                      socklen_t option_length) {
    if (fd == simulated_can_socket.load(std::memory_order_acquire)) {
        return 0;
    }
    return __real_setsockopt(fd, level, option_name, option_value, option_length);
}

int __real_close(int fd);
int __wrap_close(int fd) {
    if (fd == simulated_can_socket.load(std::memory_order_acquire)) {
        simulated_can_socket.store(-1, std::memory_order_release);
    }
    if (delay_send_after_select.load(std::memory_order_acquire) &&
        fd == delayed_send_socket.load(std::memory_order_acquire)) {
        delayed_send_socket_closed.store(true, std::memory_order_release);
        delayed_send_close_entered.store(true, std::memory_order_release);
    }
    if (delay_read_after_select.load(std::memory_order_acquire) &&
        fd == delayed_read_socket.load(std::memory_order_acquire)) {
        delayed_read_socket_closed.store(true, std::memory_order_release);
        delayed_read_close_entered.store(true, std::memory_order_release);
    }
    if (pause_write_before_syscall.load(std::memory_order_acquire) &&
        fd == paused_write_fd.load(std::memory_order_acquire)) {
        paused_write_fd_closed.store(true, std::memory_order_release);
        paused_write_close_entered.store(true, std::memory_order_release);
    }
    return __real_close(fd);
}

int __real_select(int nfds, fd_set* read_fds, fd_set* write_fds, fd_set* except_fds,
                  timeval* timeout);
int __wrap_select(int nfds, fd_set* read_fds, fd_set* write_fds, fd_set* except_fds,
                  timeval* timeout) {
    const int failing_read_fd = fail_next_read_select_fd.load(std::memory_order_acquire);
    if (read_fds != nullptr && failing_read_fd >= 0 && FD_ISSET(failing_read_fd, read_fds) &&
        read_select_failure_injected.load(std::memory_order_acquire) &&
        pause_read_select_after_failure.load(std::memory_order_acquire)) {
        read_select_paused_after_failure.store(true, std::memory_order_release);
        while (!release_read_select_after_failure.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    bool should_fail = true;
    if (read_fds != nullptr && failing_read_fd >= 0 && FD_ISSET(failing_read_fd, read_fds) &&
        fail_next_read_select.compare_exchange_strong(should_fail, false, std::memory_order_acq_rel)) {
        read_select_failure_injected.store(true, std::memory_order_release);
        errno = EBADF;
        return -1;
    }

    const int result = __real_select(nfds, read_fds, write_fds, except_fds, timeout);
    const int fd = delayed_send_socket.load(std::memory_order_acquire);
    if (result > 0 && delay_send_after_select.load(std::memory_order_acquire) &&
        write_fds != nullptr && fd >= 0 && FD_ISSET(fd, write_fds)) {
        send_select_returning.store(true, std::memory_order_release);
        while (!release_send_select.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    const int read_fd = delayed_read_socket.load(std::memory_order_acquire);
    if (result > 0 && delay_read_after_select.load(std::memory_order_acquire) &&
        read_fds != nullptr && read_fd >= 0 && FD_ISSET(read_fd, read_fds)) {
        read_select_returning.store(true, std::memory_order_release);
        while (!release_read_select.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    return result;
}

ssize_t __real_write(int fd, const void* buffer, size_t size);
ssize_t __wrap_write(int fd, const void* buffer, size_t size) {
    if (capture_next_write_fd.exchange(false, std::memory_order_acq_rel)) {
        paused_write_fd.store(fd, std::memory_order_release);
    }
    if (delay_send_after_select.load(std::memory_order_acquire) &&
        fd == delayed_send_socket.load(std::memory_order_acquire) &&
        delayed_send_socket_closed.load(std::memory_order_acquire)) {
        writes_after_send_socket_close.fetch_add(1, std::memory_order_relaxed);
    }
    if (pause_write_before_syscall.load(std::memory_order_acquire) &&
        fd == paused_write_fd.load(std::memory_order_acquire)) {
        paused_write_entered.store(true, std::memory_order_release);
        while (!release_paused_write.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        if (paused_write_fd_closed.load(std::memory_order_acquire)) {
            writes_after_paused_fd_close.fetch_add(1, std::memory_order_relaxed);
        }
    }
    const size_t max_bytes = next_write_max_bytes.exchange(0, std::memory_order_acq_rel);
    if (max_bytes > 0 && max_bytes < size) {
        size = max_bytes;
    }
    return __real_write(fd, buffer, size);
}

ssize_t __real_read(int fd, void* buffer, size_t size);
ssize_t __wrap_read(int fd, void* buffer, size_t size) {
    const int failing_read_fd = fail_next_read_fd.load(std::memory_order_acquire);
    if (fd == failing_read_fd &&
        read_failure_injected.load(std::memory_order_acquire) &&
        pause_read_after_failure.load(std::memory_order_acquire)) {
        read_paused_after_failure.store(true, std::memory_order_release);
        while (!release_read_after_failure.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    bool should_fail = true;
    if (fd == failing_read_fd &&
        fail_next_read.compare_exchange_strong(should_fail, false, std::memory_order_acq_rel)) {
        read_failure_injected.store(true, std::memory_order_release);
        errno = EBADF;
        return -1;
    }
    if (delay_read_after_select.load(std::memory_order_acquire) &&
        fd == delayed_read_socket.load(std::memory_order_acquire) &&
        delayed_read_socket_closed.load(std::memory_order_acquire)) {
        reads_after_read_socket_close.fetch_add(1, std::memory_order_relaxed);
    }
    if (pause_read_before_syscall.load(std::memory_order_acquire) &&
        fd == paused_read_fd.load(std::memory_order_acquire)) {
        paused_read_entered.store(true, std::memory_order_release);
        while (!release_paused_read.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
    }
    return __real_read(fd, buffer, size);
}

}  // extern "C"

#endif
