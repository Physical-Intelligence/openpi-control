/*!
 * @file pi_control_node.cpp
 * @brief Main entry point for the robot device control application.
 */

#include <algorithm>
#include <boost/program_options.hpp>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pi_command_line_args.hpp"
#include "pi_control.hpp"
#include "pi_device.hpp"
#include "pi_device_config.hpp"
#include "pi_exception.hpp"
#include "pi_profile.hpp"

volatile std::sig_atomic_t g_terminate_signal_received = 0;

void pi_control_signal_handler(int signum) {
    g_terminate_signal_received = signum;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, pi_control_signal_handler);
    std::signal(SIGHUP, pi_control_signal_handler);
    std::signal(SIGTERM, pi_control_signal_handler);

    // Own the device instance for the entire scope of main (including catch
    // blocks).
    std::unique_ptr<Device> p_device;

    try {
        CommandLineArgs cla(argc, argv);

        g_info_manager.set_info_level((InfoLevel)cla.info_level);
        g_info_manager.add_groups(cla.info_groups, ',');

        ReturnCode return_code;

        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Processing configuration files...");

        DeviceConfig device_config_model;
        return_code = device_config_model.init_config_model(cla);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to read device model configuration");
            return -1;
        }

        DeviceConfig device_config_individual;
        return_code = device_config_individual.init_config_individual(cla);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to read device individual configuration");
            return -1;
        }

        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Creating device...");

        p_device.reset(Device::new_device(device_config_model,
                                          device_config_individual, cla));
        if (!p_device) {
            PI_ERROR("Failed to create device");
            return -1;
        }

        p_device->set_topic_joystick_name(cla.topic_joystick);

        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Initializing device...");

        return_code = p_device->init(cla, argc, argv);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Device initialization failed");
            return -1;
        }

        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Starting devices...");

        return_code = p_device->start(0);
        if (return_code != ReturnCode::SUCCESS) {
            // Communication failures should not keep the process running for
            // long.
            PI_ERROR("Device start() failed: error code=%d", return_code);
            return -1;
        }

        int capability_flags = PI_CONTROL_CAP_MOVE_TO_READY;
        if (cla.role == Role::FOLLOWER) {
            capability_flags |= PI_CONTROL_CAP_DIRECT_COMMAND | PI_CONTROL_CAP_LIVE_INPUT;
        } else {
            capability_flags |= PI_CONTROL_CAP_GRAVITY_COMP | PI_CONTROL_CAP_FORCE_FEEDBACK;
        }
        std::vector<int> handshake_data{
            PI_CONTROL_PROTOCOL_VERSION_MAJOR,
            PI_CONTROL_PROTOCOL_VERSION_MINOR,
            capability_flags,
        };

        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Starting main control loop...");

        bool informed_ready_now = false;
        // ZMQ PUB/SUB can drop early messages until subscribers finish
        // connecting ("slow joiner"). To make readiness robust, re-publish
        // DEVICE_INFO_READY_NOW for a short period after first ready.
        int ready_announce_loops_remaining = 0;
        const int ready_announce_total_loops =
            std::max(1, (int)cla.control_frequency);  // ~1 second
        const int ready_announce_interval_loops =
            std::max(1, (int)cla.control_frequency / 10);  // ~0.1 sec
        int handshake_announce_counter = 0;

        while (g_terminate_signal_received == 0 && p_device->is_running()) {
            return_code = p_device->step();
            if (return_code != ReturnCode::SUCCESS) {
                if (return_code == ReturnCode::HARDWARE_FAULT) {
                    PI_ERROR("Protective stop: servo reported a hardware fault; "
                             "see preceding HARDWARE FAULT message");
                } else if (return_code <= ReturnCode::SAFE_MODE) {
                    if (return_code == ReturnCode::SAFE_MODE_POS_BEHIND) {
                        PI_ERROR(
                            "Protective stop: Position target behind limit "
                            "triggered safe mode");
                    } else if (return_code ==
                               ReturnCode::SAFE_MODE_POS_EXCEED) {
                        PI_ERROR(
                            "Protective stop: Position limit exceeded "
                            "triggered safe mode");
                    } else if (return_code == ReturnCode::SAFE_MODE_VEL) {
                        PI_ERROR(
                            "Protective stop: Velocity limit exceeded "
                            "triggered safe mode");
                    } else if (return_code == ReturnCode::SAFE_MODE_TOR) {
                        PI_ERROR(
                            "Protective stop: Torque limit exceeded triggered "
                            "safe mode");
                    } else if (return_code == ReturnCode::SAFE_MODE_SIG) {
                        PI_ERROR(
                            "Protective stop: Servo signal loss triggered safe "
                            "mode");
                    } else if (return_code ==
                               ReturnCode::SAFE_MODE_TEMPERATURE) {
                        PI_ERROR(
                            "Protective stop: Temperature limit exceeded "
                            "triggered safe mode");
                    } else {
                        PI_ERROR("Protective stop: Unknown error code: %d",
                                 return_code);
                    }
                } else {
                    PI_WARN("Device step() failed: error code=%d", return_code);
                }

                // Graceful recovery: instead of breaking immediately (which would torque-off all
                // joints at the current pose and let a heavy arm fall), trigger the emergency
                // recovery state machine. The device will switch to a follower-like position
                // mode and slowly drive reachable joints to the ready position at ERROR speed
                // before the topic self-stops via mark_emergency_recovery_completed(). This is
                // the only failure path for joint errors; there is no opt-out for the legacy
                // immediate-park behavior (it was unsafe at any distance from home).
                if (!p_device->is_in_emergency_recovery()) {
                    // Use the joint id recorded by the device's read_hardware_values() path
                    // (set via set_last_failed_joint_id) so the UI dialog can name the
                    // specific failed joint instead of showing "joint -1".
                    const int failed_joint_id = p_device->last_failed_joint_id();
                    p_device->enter_emergency_recovery(return_code, failed_joint_id);
                    // Continue stepping; do NOT break.
                } else {
                    // We already entered recovery and this iteration still failed. That is expected
                    // (e.g. continued joint drop-outs); the slow ready move keeps trying best-effort.
                    PI_WARN("Error during emergency recovery (rc=%d); continuing best-effort",
                            static_cast<int>(return_code));
                }
            }

            if ((handshake_announce_counter++ % std::max(1, cla.control_frequency / 2)) == 0) {
                p_device->publish_device_info(DEVICE_INFO_PROTOCOL_HANDSHAKE, nullptr, &handshake_data);
            }

            const bool ready_now = p_device->is_ready();
            if (!ready_now) {
                // Allow re-announcement when the device re-enters ready state
                // (e.g. after COMMAND_MOVE_TO_READY_POS).
                informed_ready_now = false;
            }

            if (ready_now) {
                if (informed_ready_now == false) {
                    // Start re-announcement window.
                    ready_announce_loops_remaining = ready_announce_total_loops;
                    informed_ready_now = true;
                }

                // Publish at the first ready tick and then periodically for ~1
                // second.
                if (ready_announce_loops_remaining > 0) {
                    const bool is_first_ready_publish =
                        (ready_announce_loops_remaining ==
                         ready_announce_total_loops);
                    const bool is_periodic_publish =
                        ((ready_announce_loops_remaining %
                          ready_announce_interval_loops) == 0);
                    if (is_first_ready_publish || is_periodic_publish) {
                        std::vector<int> ready_data;
                        const int completed_request_id =
                            p_device->completed_move_to_ready_request_id();
                        if (completed_request_id > 0) {
                            ready_data.push_back(completed_request_id);
                        }
                        return_code = p_device->publish_device_info(
                            DEVICE_INFO_READY_NOW, nullptr,
                            ready_data.empty() ? nullptr : &ready_data);
                        if (return_code != ReturnCode::SUCCESS) {
                            PI_ERROR(
                                "Failed to publish device ready status: error "
                                "code=%d",
                                return_code);
                            break;
                        }
                        if (is_first_ready_publish) {
                            PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                                    "Device is ready: %s_%s",
                                    p_device->get_model().c_str(),
                                    p_device->get_id().c_str());
                        }
                    }
                    ready_announce_loops_remaining -= 1;
                }
            }

            if (!p_device) {
                PI_ERROR("Device pointer is null in main control loop");
                break;
            }
            p_device->sleep();
        }

        const std::sig_atomic_t terminate_signal = g_terminate_signal_received;
        if (terminate_signal != 0) {
            PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                    "Signal handler called with signal: %d",
                    static_cast<int>(terminate_signal));
        }

        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Main control loop ended, shutting down device...");

        if (p_device) {
            PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Stopping device...");
            p_device->stop();
            p_device.reset();
        }
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Device stopped");

    } catch (const PiException& e) {
        std::cerr << "Caught PiException: " << e.what() << std::endl;

        ///< @todo Disabled throwing exception in PI_ERROR() to avoid abrupt
        ///< program termination

        if (p_device) {
            p_device->park_safely();
            p_device.reset();
        }

        return 1;

    } catch (const std::exception& e) {
        std::cerr << "Caught standard exception: " << e.what() << std::endl;

        // Check if the exception is related to "Address already in use" or ZMQ
        // binding
        std::string error_msg = e.what();
        if (error_msg.find("Address already in use") != std::string::npos ||
            error_msg.find("EADDRINUSE") != std::string::npos ||
            error_msg.find("ZMQ bind failed") != std::string::npos ||
            error_msg.find("ZMQ connect failed") != std::string::npos) {
            PI_ERROR("ZMQ address binding/connection failed");
            PI_ERROR("Error details: %s", error_msg.c_str());
            PI_ERROR(
                "This usually means a ZMQ port is already bound by another "
                "process");
            PI_ERROR(
                "Try checking if another pi_control_node instance is running");
            PI_ERROR(
                "Or check whether the ZMQ ports are still in use: ss -ltnp");
        } else {
            // For other exceptions, also try to extract address information if
            // available
            PI_ERROR("Standard exception occurred: %s", error_msg.c_str());
        }

        if (p_device) {
            p_device->park_safely();
            p_device.reset();
        }

        return 1;

    } catch (...) {
        std::cerr << "Caught unknown exception" << std::endl;
        if (p_device) {
            p_device->park_safely();
            p_device.reset();
        }

        return 1;
    }

    PI_INFO("main()", InfoLevel::ESSENTIAL_0,
            "pi_control_node terminated successfully");

    return 0;
}
