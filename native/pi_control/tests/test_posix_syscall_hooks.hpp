#pragma once

#if defined(__linux__)

#include <atomic>
#include <chrono>

namespace test_posix_hooks {

extern std::atomic<bool> simulate_can_open;
extern std::atomic<int> simulated_can_socket;

extern std::atomic<int> delayed_send_socket;
extern std::atomic<bool> delay_send_after_select;
extern std::atomic<bool> send_select_returning;
extern std::atomic<bool> release_send_select;
extern std::atomic<bool> delayed_send_socket_closed;
extern std::atomic<bool> delayed_send_close_entered;
extern std::atomic<int> writes_after_send_socket_close;

extern std::atomic<int> delayed_read_socket;
extern std::atomic<bool> delay_read_after_select;
extern std::atomic<bool> read_select_returning;
extern std::atomic<bool> release_read_select;
extern std::atomic<bool> delayed_read_socket_closed;
extern std::atomic<bool> delayed_read_close_entered;
extern std::atomic<int> reads_after_read_socket_close;

extern std::atomic<int> fail_next_read_select_fd;
extern std::atomic<bool> fail_next_read_select;
extern std::atomic<bool> read_select_failure_injected;
extern std::atomic<bool> pause_read_select_after_failure;
extern std::atomic<bool> read_select_paused_after_failure;
extern std::atomic<bool> release_read_select_after_failure;

extern std::atomic<int> fail_next_read_fd;
extern std::atomic<bool> fail_next_read;
extern std::atomic<bool> read_failure_injected;
extern std::atomic<bool> pause_read_after_failure;
extern std::atomic<bool> read_paused_after_failure;
extern std::atomic<bool> release_read_after_failure;

extern std::atomic<int> paused_read_fd;
extern std::atomic<bool> pause_read_before_syscall;
extern std::atomic<bool> paused_read_entered;
extern std::atomic<bool> release_paused_read;

extern std::atomic<int> paused_write_fd;
extern std::atomic<bool> capture_next_write_fd;
extern std::atomic<size_t> next_write_max_bytes;
extern std::atomic<bool> pause_write_before_syscall;
extern std::atomic<bool> paused_write_entered;
extern std::atomic<bool> release_paused_write;
extern std::atomic<bool> paused_write_fd_closed;
extern std::atomic<bool> paused_write_close_entered;
extern std::atomic<int> writes_after_paused_fd_close;

bool wait_for_flag(const std::atomic<bool>& flag, std::chrono::milliseconds timeout);

}  // namespace test_posix_hooks

#endif
