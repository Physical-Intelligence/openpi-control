/*!
 * @file pi_driver.hpp
 * @brief Driver abstract base class for hardware communication interfaces.
 */

#pragma once
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "pi_control.hpp"
#include "pi_device_config.hpp"

class Servo;
class ServoDm;
class Device;

/*!
 * @brief Abstract base class for hardware communication drivers.
 */
class Driver {
   public:
    /*!
     * @brief Function pointer type for finding data index from servo ID.
     * @param servo_id The servo ID to look up.
     * @return The data index corresponding to the servo ID, or -1 if not found.
     */
    typedef int (*func_find_data_index_t)(int servo_id);

    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments.
     */
    explicit Driver(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    virtual ~Driver() = default;

    /*!
     * @brief Opens the communication port for hardware communication.
     * @param baud_rate Retained driver API parameter; ignored by SocketCAN.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode open(int baud_rate) = 0;

    /*!
     * @brief Closes the communication port.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode close() = 0;

    /*!
     * @brief Reads current hardware values from a single servo.
     * @param p_servo Pointer to the Servo instance.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode read_hardware_values(Servo* p_servo);

    /*!
     * @brief Reads hardware values from all servos using group read operation.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode group_read_hardware_values() { return ReturnCode::SUCCESS; }

    /*!
     * @brief Returns the lowest servo id in ``dead_servo_ids()`` (the cable-
     *        disconnection point on a daisy-chained bus), or -1 if every servo
     *        responded. Convenience accessor for Device code that wants to
     *        report a single id to the UI.
     */
    virtual int last_failed_servo_id() const { return -1; }

    /*!
     * @brief Returns a snapshot of the servo ids currently considered dead on
     *        the bus. On a daisy-chained bus a single cable disconnection
     *        knocks out every servo downstream of the break, so the set
     *        generally contains more than one id. Device implementations use
     *        this to mark every dead joint and to skip per-joint read/write
     *        attempts on them during emergency recovery.
     *
     *        The set is sticky across cycles -- once a servo is detected as
     *        dead the driver also removes it from any group_read/group_write
     *        id list so subsequent bulk transactions only talk to the alive
     *        servos. Drivers that don't track this return an empty set.
     */
    virtual std::set<int> dead_servo_ids() const { return {}; }

    /*!
     * @brief Writes hardware values to all servos using group write operation.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode group_write_hardware_values() { return ReturnCode::SUCCESS; }

    /*!
     * @brief Resets the zero position (encoder offset) of a servo motor.
     * @param id Servo ID of the servo.
     * @param type Servo type or model identifier.
     * @return ReturnCode::SUCCESS if successful (default implementation).
     */
    virtual ReturnCode reset_zero_position(int id, int type) {
        (void)id;
        (void)type;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Registers a servo's ID, data index, and pointer in the static mapping tables.
     * @param servo_id Servo ID.
     * @param servo_data_index Data index for the servo in group read/write buffers.
     * @param p_servo Pointer to the Servo instance.
     */
    static void register_servo_data_index(int servo_id, int servo_data_index, class Servo* p_servo) {
        std::lock_guard<std::mutex> lock(registry_mutex_);
        map_id_to_data_index_[servo_id] = servo_data_index;
        map_id_to_servo_[servo_id] = p_servo;
    }

    /*!
     * @brief Finds the data index for a servo given its ID.
     * @param servo_id Servo ID to look up.
     * @return The data index if found, or -1 if not found.
     */
    static int find_data_index(int servo_id);

    /*!
     * @brief Finds the Servo pointer for a servo given its ID.
     * @param servo_id Servo ID to look up.
     * @return Pointer to the Servo instance if found, or nullptr if not found.
     */
    static class Servo* find_servo(int servo_id);

    /*!
     * @brief Factory method to create a new Driver instance.
     * @param p_device Pointer to the Device instance.
     * @param p_config Pointer to the DeviceConfig instance.
     * @param cla Command-line arguments.
     * @return Pointer to the newly created Driver instance.
     */
    static std::shared_ptr<Driver> new_driver(Device* p_device, const DeviceConfig* p_config, const CommandLineArgs& cla);

   protected:
    std::string control_port_name_;  ///< Name of the control port.
    Device* p_device_ = nullptr;  ///< Pointer to the Device instance.
    inline static std::mutex registry_mutex_;  ///< Guards the process-wide servo registry maps.
    static std::map<int, int> map_id_to_data_index_;  ///< Static map from servo ID to data index.
    static std::map<int, class Servo*> map_id_to_servo_;  ///< Static map from servo ID to Servo pointer.

   private:
    friend class Servo;
    friend class ServoDm;

    class RegisteredServo {
       public:
        RegisteredServo(RegisteredServo&&) = default;
        class Servo* get() const { return p_servo_; }

       private:
        friend class Driver;
        RegisteredServo(std::unique_lock<std::mutex>&& lock, class Servo* p_servo)
            : lock_(std::move(lock)), p_servo_(p_servo) {}

        std::unique_lock<std::mutex> lock_;
        class Servo* p_servo_;
    };

    static void unregister_servo(class Servo* p_servo);
    static RegisteredServo lock_registered_servo(int servo_id);
};
