/*!
 * @file pi_driver_serial.hpp
 * @brief DriverSerial -- minimal raw POSIX serial (termios) wrapper.
 *
 * Designed for protocols that ride a plain USB-CDC `/dev/ttyACM*` port and do
 * their own framing. This driver knows nothing about a specific servo/motor
 * wire format -- it only owns the file descriptor, sets 8N1 with the
 * requested baud rate, and gives the device class raw byte access plus an
 * optional asynchronous reception thread. Protocol drivers (e.g.
 * :class:`DriverFt` for FeeTech SMS/STS, a future Dynamixel driver) derive
 * from it and implement their packet layer on top.
 */

#pragma once
#include <termios.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <thread>

#include "pi_driver.hpp"

/*!
 * @brief Minimal raw serial driver (no framing, no protocol).
 */
class DriverSerial : public Driver {
   public:
    /*!
     * @brief Callback type for received serial bytes.
     * @param p_data_buf Pointer to the receive buffer.
     * @param read_bytes Number of bytes actually read.
     */
    typedef std::function<void(const uint8_t* p_data_buf, size_t read_bytes)> callback_t;

    DriverSerial(Device* p_device, const CommandLineArgs& cla);
    ~DriverSerial() override;

    /*!
     * @brief Open the serial port at ``baud_rate`` (8N1, no flow control).
     *
     * The port path comes from the base class' ``control_port_name_`` (set
     * by the ``Driver`` constructor from ``cla.control_port_name``).
     */
    ReturnCode open(int baud_rate) override;
    ReturnCode close() override;

    /*!
     * @brief Start a background thread that drains the serial port and
     *        invokes ``callback`` for every chunk of bytes read.
     *
     * Idempotent: stops a previous reception loop first if one is running.
     */
    ReturnCode start_reception(const callback_t& callback);

    /*!
     * @brief Stop the reception thread (no-op if not running).
     */
    ReturnCode stop_reception();

    /*!
     * @brief Write raw bytes to the serial port. Blocking ``::write`` semantics.
     */
    ReturnCode write_bytes(const uint8_t* p_data, size_t size);

    /*!
     * @brief Convenience overload: write a NUL-terminated text command.
     */
    ReturnCode write_text(const char* text);

    /*!
     * @brief Whether ``open()`` succeeded and we still own a valid fd.
     */
    bool is_open() const { return port_handler_ >= 0; }

   protected:
    void receive_loop(callback_t callback);

    /*!
     * @brief Map a baud-rate integer to the matching termios ``B*`` constant.
     * @return The mapped speed_t, or 0 if ``baud_rate`` is unsupported.
     */
    static speed_t baud_constant(int baud_rate);

    int port_handler_ = -1;          ///< POSIX file descriptor (-1 when closed).
    std::atomic<bool> is_running_{false};
    std::thread reception_thread_;
};
