/*!
 * @file pi_driver_can.hpp
 * @brief DriverCan base class for CAN bus communication.
 */

#pragma once
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "pi_driver.hpp"

/*!
 * @brief Base class for CAN bus communication drivers.
 */
class DriverCan : public Driver {
   public:
    /*!
     * @brief Type alias for standard CAN frame structure.
     */
    typedef struct can_frame can_frame_t;

    /*!
     * @brief Callback function type for processing received CAN messages.
     * @param p_data_buf Pointer to the received CAN frame data.
     * @param data_buf_size Size of the data buffer in bytes.
     * @param read_bytes Number of bytes actually read from the socket.
     */
    typedef std::function<void(void* p_data_buf, size_t data_buf_size, size_t read_bytes)> callback_t;

    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments.
     */
    DriverCan(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DriverCan();


    /*!
     * @brief Opens the CAN control port and binds it to the CAN interface.
     * @param baud_rate Baud rate parameter (unused for CAN).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode open(int baud_rate) override;

    /*!
     * @brief Closes the CAN control port and stops message reception.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode close() override;

    /*!
     * @brief Starts asynchronous reception of CAN messages with callback handling.
     * @param callback Callback function that will be invoked for each received CAN message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode start_reception(const callback_t& callback);

    /*!
     * @brief Stops asynchronous reception of CAN messages.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode stop_reception();

    /*!
     * @brief Sends a classic CAN frame over the CAN bus.
     * @param p_data_buf Pointer to the CAN frame data buffer.
     * @param data_buf_size Size of the data buffer in bytes.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode send_frame(void* p_data_buf, size_t data_buf_size);

    /*!
     * @brief Reads a single classic CAN frame from the CAN bus (synchronous).
     * @param p_data_buf Pointer to the data buffer where the received CAN frame will be stored.
     * @param data_buf_size Size of the data buffer in bytes.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode read_frame(void* p_data_buf, size_t data_buf_size);

   protected:
    bool is_socket_open() {
        std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
        return sock_ >= 0;
    }

    /*!
     * @brief Main loop function for the reception thread.
     * @param callback Callback function that will be invoked for each received CAN message.
     */
    void receive_loop(const callback_t& callback);

    std::string port_name_;  ///< Name of the CAN interface (e.g., "can0", "can1").
    int sock_ = -1;  ///< File descriptor for the CAN socket.
    struct sockaddr_can addr_{};  ///< Socket address structure for the CAN interface. Value-initialized:
                                  ///< open() only sets can_family/can_ifindex, but bind() reads the whole
                                  ///< struct including the can_addr union and padding.
    std::atomic<bool> is_running_;  ///< Atomic flag indicating whether the reception thread is running.
    std::thread reception_thread_;  ///< Background thread that runs receive_loop().
    void* p_data_buf_;  ///< Pointer to the data buffer used for receiving CAN messages.
    size_t data_buf_size_;  ///< Size of the data buffer in bytes.
    can_frame_t data_frame_;  ///< Standard CAN frame structure.

   private:
    bool wait_for_reception_join_locked(std::unique_lock<std::mutex>& lifecycle_lock);
    void join_reception_thread_locked(std::unique_lock<std::mutex>& lifecycle_lock);
    ReturnCode close_locked(std::unique_lock<std::mutex>& lifecycle_lock);
    ReturnCode stop_reception_locked(std::unique_lock<std::mutex>& lifecycle_lock);

    std::mutex lifecycle_mutex_;
    std::condition_variable lifecycle_cv_;
    bool reception_join_in_progress_ = false;
    std::thread::id reception_thread_id_;
};
