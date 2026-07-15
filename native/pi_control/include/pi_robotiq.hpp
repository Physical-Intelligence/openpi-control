#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

struct RobotiqConfig {
    std::string device;
    int baud_rate = 115200;
    int slave_id = 9;
    int poll_frequency_hz = 50;
    uint8_t open_raw = 3;
    uint8_t closed_raw = 230;
};

struct RobotiqState {
    bool connected = false;
    bool activated = false;
    bool moving = false;
    float position = 1.0f;  // 0 closed, 1 open.
    float velocity = 0.0f;
    float effort = 0.0f;
    float current = 0.0f;
    float target = 1.0f;
    uint8_t fault = 0;
};

class RobotiqTransport {
   public:
    explicit RobotiqTransport(RobotiqConfig config);
    ~RobotiqTransport();

    bool start();
    void stop();
    bool activate();
    void set_target(float position, float speed, float force);
    void hold();
    RobotiqState state() const;

    static uint8_t position_to_raw(float normalized_open, uint8_t open_raw, uint8_t closed_raw);
    static float raw_to_position(uint8_t raw, uint8_t open_raw, uint8_t closed_raw);
    static uint8_t normalized_to_raw(float value, bool preserve_regrasp = false);

   private:
    struct Impl;
    void run();

    RobotiqConfig config_;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    RobotiqState state_;
    float target_position_ = 1.0f;
    float target_speed_ = 1.0f;
    float target_force_ = 1.0f;
    uint64_t command_generation_ = 0;
    bool activation_requested_ = false;
    bool hold_requested_ = false;
};
