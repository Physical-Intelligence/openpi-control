/*!
 * @file pi_driver_serial.cpp
 * @brief Implementation of the DriverSerial raw-tty wrapper.
 */

#include "pi_driver_serial.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <unistd.h>

#if defined(__linux__)
#include <linux/serial.h>
#elif defined(__APPLE__)
#include <IOKit/serial/ioss.h>
#endif

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <limits.h>
#include <map>
#include <string>

#include "pi_info.hpp"

namespace {

std::string tty_name_from_path(const std::string& path) {
    char resolved[PATH_MAX] = {0};
    const char* p = ::realpath(path.c_str(), resolved);
    const std::string full = (p != nullptr) ? std::string(resolved) : path;
    const size_t slash = full.find_last_of('/');
    return slash == std::string::npos ? full : full.substr(slash + 1);
}

bool file_exists(const std::string& path) {
    struct stat st {};
    return ::stat(path.c_str(), &st) == 0;
}

bool write_text_file(const std::string& path, const std::string& value) {
    std::ofstream f(path);
    if (!f.is_open()) {
        return false;
    }
    f << value;
    return f.good();
}

void apply_usb_serial_latency_timer(const std::string& port_path) {
#if defined(__linux__)
    const std::string tty = tty_name_from_path(port_path);
    if (tty.empty()) {
        return;
    }

    const std::string candidates[] = {
        "/sys/class/tty/" + tty + "/device/latency_timer",
        "/sys/bus/usb-serial/devices/" + tty + "/latency_timer",
    };
    bool saw_latency_timer = false;
    for (const std::string& latency_path : candidates) {
        if (!file_exists(latency_path)) {
            continue;
        }
        saw_latency_timer = true;
        // FTDI latency_timer accepts 1..255 ms; 0 is not a valid sysfs value on
        // normal Linux ftdi_sio, so 1 ms is the practical minimum here.
        if (write_text_file(latency_path, "1\n")) {
            PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
                    "DriverSerial: set USB serial latency_timer=1 at %s",
                    latency_path.c_str());
        } else {
            PI_WARN("DriverSerial: failed to set USB serial latency_timer at %s: %s",
                    latency_path.c_str(), std::strerror(errno));
        }
    }
    if (!saw_latency_timer) {
        PI_INFO("Driver", InfoLevel::FREQUENT_3,
                "DriverSerial: no USB serial latency_timer sysfs entry for %s (%s)",
                port_path.c_str(), tty.c_str());
    }
#else
    // macOS uses IOSSDATALAT (per-fd ioctl) applied below in apply_low_latency_flag().
    (void)port_path;
#endif
}

void apply_low_latency_flag(int fd, const std::string& port_path) {
#if defined(__linux__)
    serial_struct serial {};
    if (::ioctl(fd, TIOCGSERIAL, &serial) != 0) {
        PI_INFO("Driver", InfoLevel::FREQUENT_3,
                "DriverSerial: TIOCGSERIAL low-latency query unsupported for '%s': %s",
                port_path.c_str(), std::strerror(errno));
        return;
    }
    if ((serial.flags & ASYNC_LOW_LATENCY) != 0) {
        return;
    }
    serial.flags |= ASYNC_LOW_LATENCY;
    if (::ioctl(fd, TIOCSSERIAL, &serial) != 0) {
        PI_WARN("DriverSerial: failed to set ASYNC_LOW_LATENCY for '%s': %s",
                port_path.c_str(), std::strerror(errno));
        return;
    }
    PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
            "DriverSerial: enabled ASYNC_LOW_LATENCY for '%s'", port_path.c_str());
#elif defined(__APPLE__)
    // IOSSDATALAT: per-fd minimum latency in microseconds. 1 us is the
    // documented minimum and the FTDI driver clamps low values internally.
    // This is the macOS counterpart of Linux's ASYNC_LOW_LATENCY.
    unsigned long latency_us = 1;
    if (::ioctl(fd, IOSSDATALAT, &latency_us) != 0) {
        PI_INFO("Driver", InfoLevel::FREQUENT_3,
                "DriverSerial: IOSSDATALAT low-latency query unsupported for '%s': %s",
                port_path.c_str(), std::strerror(errno));
        return;
    }
    PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
            "DriverSerial: enabled IOSSDATALAT low latency for '%s'", port_path.c_str());
#else
    (void)fd;
    (void)port_path;
#endif
}

#if defined(__APPLE__)
// Apply an arbitrary baud rate on macOS via IOSSIOSPEED. This is the same
// mechanism pyserial uses. Required because Apple termios B<rate> macros
// stop at B230400; our Dynamixel servos need 1 Mbps / 3 Mbps.
bool apply_mac_custom_baud(int fd, int baud_rate) {
    speed_t speed = static_cast<speed_t>(baud_rate);
    return ::ioctl(fd, IOSSIOSPEED, &speed) == 0;
}
#endif

}  // namespace

DriverSerial::DriverSerial(Device* p_device, const CommandLineArgs& cla)
    : Driver(p_device, cla) {}

DriverSerial::~DriverSerial() {
    stop_reception();
    DriverSerial::close();
}

speed_t DriverSerial::baud_constant(int baud_rate) {
    // Apple <termios.h> only defines B-prefixed macros up to B230400. Higher
    // rates (460800 / 500000 / 921600 / 1 Mbps) ship as Linux extensions in
    // <bits/termios-baud.h> and are absent on macOS. Each high-rate entry is
    // therefore guarded by its own #if defined(B...) so the same source builds
    // on both platforms; macOS falls back to IOSSIOSPEED for those values
    // (see DriverSerial::open() — the speed==0 branch on Apple).
    static const std::map<int, speed_t> kMap = {
        {9600, B9600},     {19200, B19200},   {38400, B38400},   {57600, B57600},
        {115200, B115200}, {230400, B230400},
#if defined(B460800)
        {460800, B460800},
#endif
#if defined(B500000)
        {500000, B500000},
#endif
#if defined(B576000)
        {576000, B576000},
#endif
#if defined(B921600)
        {921600, B921600},
#endif
#if defined(B1000000)
        {1000000, B1000000},
#endif
    };
    auto it = kMap.find(baud_rate);
    return (it == kMap.end()) ? static_cast<speed_t>(0) : it->second;
}

ReturnCode DriverSerial::open(int baud_rate) {
    if (port_handler_ >= 0) {
        // Already open -- treat as success so callers can safely retry.
        return ReturnCode::SUCCESS;
    }

    if (control_port_name_.empty()) {
        PI_ERROR("DriverSerial::open: empty control_port_name_ (set --control_port)");
        return ReturnCode::INVALID_PARAM;
    }

#if defined(__APPLE__)
    // macOS exposes every USB-serial under two synonyms: ``/dev/tty.<name>``
    // (POSIX dial-in — blocks on DCD until the modem asserts carrier) and
    // ``/dev/cu.<name>`` (call-up — opens immediately). USB-serial adapters
    // never assert DCD, so opening ``tty.usbserial-*`` or ``tty.usbmodem*``
    // hangs ``open()`` forever. Fail fast with a hint instead of letting
    // the next O_NONBLOCK ``::open()`` call block the entire process.
    if (control_port_name_.rfind("/dev/tty.usbserial", 0) == 0 ||
        control_port_name_.rfind("/dev/tty.usbmodem", 0) == 0 ||
        control_port_name_.rfind("/dev/tty.SLAB_USBtoUART", 0) == 0) {
        std::string cu_hint = control_port_name_;
        cu_hint.replace(0, 9, "/dev/cu.");  // "/dev/tty." -> "/dev/cu."
        PI_ERROR("DriverSerial::open: refusing macOS DCD-blocking path '%s'. "
                 "Use the call-up variant '%s' instead.",
                 control_port_name_.c_str(), cu_hint.c_str());
        return ReturnCode::INVALID_PARAM;
    }
#endif

    const speed_t speed = baud_constant(baud_rate);
#if defined(__APPLE__)
    // macOS termios maxes out at B230400 (B460800 / B921600 / B1000000 are
    // absent from <termios.h>). For anything above that we run cfset*speed()
    // with B9600 as a placeholder and then apply the real speed via the
    // IOSSIOSPEED ioctl. baud_constant() returning 0 is therefore expected
    // for 1 Mbps / 3 Mbps / etc. on macOS and is handled below.
    const bool use_custom_baud_macos = (speed == 0);
    const speed_t termios_speed = use_custom_baud_macos ? B9600 : speed;
#else
    if (speed == 0) {
        PI_ERROR("DriverSerial::open: unsupported baud rate %d for port '%s'",
                 baud_rate, control_port_name_.c_str());
        return ReturnCode::NOT_SUPPORTED;
    }
    const speed_t termios_speed = speed;
#endif

    apply_usb_serial_latency_timer(control_port_name_);

    const int fd = ::open(control_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        PI_ERROR("DriverSerial::open: failed to open '%s': %s",
                 control_port_name_.c_str(), std::strerror(errno));
        return ReturnCode::FAIL;
    }
    apply_low_latency_flag(fd, control_port_name_);

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (::tcgetattr(fd, &tty) != 0) {
        PI_ERROR("DriverSerial::open: tcgetattr failed for '%s': %s",
                 control_port_name_.c_str(), std::strerror(errno));
        ::close(fd);
        return ReturnCode::FAIL;
    }

    ::cfsetispeed(&tty, termios_speed);
    ::cfsetospeed(&tty, termios_speed);

    // 8N1, no flow control, raw I/O.
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);  // no parity
    tty.c_cflag &= ~CSTOPB;             // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;            // no HW flow control

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    tty.c_lflag = 0;  // raw, no echo, no canonical processing
    tty.c_oflag = 0;  // raw output

    // Non-blocking semantics on read -- the reception thread polls.
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (::tcsetattr(fd, TCSANOW, &tty) != 0) {
        PI_ERROR("DriverSerial::open: tcsetattr failed for '%s': %s",
                 control_port_name_.c_str(), std::strerror(errno));
        ::close(fd);
        return ReturnCode::FAIL;
    }

#if defined(__APPLE__)
    if (use_custom_baud_macos) {
        if (!apply_mac_custom_baud(fd, baud_rate)) {
            PI_ERROR("DriverSerial::open: IOSSIOSPEED failed for '%s' @ %d bps: %s",
                     control_port_name_.c_str(), baud_rate, std::strerror(errno));
            ::close(fd);
            return ReturnCode::FAIL;
        }
        PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
                "DriverSerial: applied macOS custom baud %d via IOSSIOSPEED on '%s'",
                baud_rate, control_port_name_.c_str());
    }
#endif

    // Some USB CDC devices do not start streaming replies until the host
    // asserts DTR/RTS. Harmless for plain UARTs.
    int modem_bits = 0;
    if (::ioctl(fd, TIOCMGET, &modem_bits) == 0) {
        modem_bits |= (TIOCM_DTR | TIOCM_RTS);
        if (::ioctl(fd, TIOCMSET, &modem_bits) != 0) {
            PI_WARN("DriverSerial::open: failed to set DTR/RTS for '%s': %s",
                    control_port_name_.c_str(), std::strerror(errno));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
    } else {
        PI_WARN("DriverSerial::open: failed to read modem bits for '%s': %s",
                control_port_name_.c_str(), std::strerror(errno));
    }

    // Flush any boot-time noise the device may have queued.
    ::tcflush(fd, TCIOFLUSH);

    port_handler_ = fd;
    PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
            "Serial port opened: port=%s baud=%d", control_port_name_.c_str(), baud_rate);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverSerial::close() {
    stop_reception();
    if (port_handler_ >= 0) {
        ::close(port_handler_);
        port_handler_ = -1;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverSerial::write_bytes(const uint8_t* p_data, size_t size) {
    if (port_handler_ < 0) {
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_data == nullptr || size == 0) {
        return ReturnCode::INVALID_PARAM;
    }

    size_t written_total = 0;
    while (written_total < size) {
        const ssize_t n = ::write(port_handler_, p_data + written_total, size - written_total);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // The CDC kernel buffer is full -- yield briefly and retry so
                // we do not silently drop bytes mid-frame.
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            PI_ERROR("DriverSerial::write_bytes: write failed on '%s': %s",
                     control_port_name_.c_str(), std::strerror(errno));
            return ReturnCode::FAIL;
        }
        written_total += static_cast<size_t>(n);
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverSerial::write_text(const char* text) {
    if (text == nullptr) {
        return ReturnCode::INVALID_PARAM;
    }
    return write_bytes(reinterpret_cast<const uint8_t*>(text), std::strlen(text));
}

ReturnCode DriverSerial::start_reception(const callback_t& callback) {
    stop_reception();

    if (port_handler_ < 0) {
        PI_ERROR("DriverSerial::start_reception: port not open");
        return ReturnCode::NOT_INITIALIZED;
    }
    if (!callback) {
        PI_ERROR("DriverSerial::start_reception: empty callback");
        return ReturnCode::INVALID_PARAM;
    }

    is_running_.store(true, std::memory_order_release);
    reception_thread_ = std::thread(&DriverSerial::receive_loop, this, callback);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverSerial::stop_reception() {
    if (!is_running_.exchange(false, std::memory_order_acq_rel)) {
        if (reception_thread_.joinable()) {
            reception_thread_.join();
        }
        return ReturnCode::SUCCESS;
    }
    if (reception_thread_.joinable()) {
        reception_thread_.join();
    }
    return ReturnCode::SUCCESS;
}

void DriverSerial::receive_loop(callback_t callback) {
    uint8_t buffer[256];
    while (is_running_.load(std::memory_order_acquire)) {
        const int fd = port_handler_;
        if (fd < 0) {
            break;
        }
        const ssize_t n = ::read(fd, buffer, sizeof(buffer));
        if (n > 0) {
            callback(buffer, static_cast<size_t>(n));
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            PI_WARN("DriverSerial::receive_loop: read error on '%s': %s",
                    control_port_name_.c_str(), std::strerror(errno));
            // Stop on hard errors so the device class can react (e.g. mark not-ready).
            break;
        }
        // Cooperative idle so we do not pin a CPU on an empty CDC pipe.
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}
