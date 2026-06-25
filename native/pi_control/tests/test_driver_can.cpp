#include <gtest/gtest.h>

#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <thread>

#include "pi_driver_can.hpp"
#include "test_posix_syscall_hooks.hpp"

#if defined(__linux__)
using namespace test_posix_hooks;
#endif

namespace {

class TestableDriverCan : public DriverCan {
   public:
    using DriverCan::DriverCan;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    bool reception_running() const { return is_running_.load(std::memory_order_acquire); }
};

#if defined(__linux__)
bool child_exited_successfully(pid_t child, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    int status = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const pid_t result = waitpid(child, &status, WNOHANG);
        if (result == child) {
            return WIFEXITED(status) && WEXITSTATUS(status) == 0;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    kill(child, SIGKILL);
    waitpid(child, &status, 0);
    return false;
}
#endif

}  // namespace

TEST(DriverCanLifecycle, ConcurrentStartsAreSafe) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0) {
            _exit(2);
        }

        CommandLineArgs cla{};
        TestableDriverCan driver(nullptr, cla);
        driver.adopt_socket(sockets[0]);

        for (int round = 0; round < 10; ++round) {
            constexpr size_t kStarterCount = 64;
            std::array<std::thread, kStarterCount> starters;
            std::atomic<size_t> ready{0};
            std::atomic<bool> go{false};
            for (auto& starter : starters) {
                starter = std::thread([&] {
                    ready.fetch_add(1, std::memory_order_release);
                    while (!go.load(std::memory_order_acquire)) {
                        std::this_thread::yield();
                    }
                    if (driver.start_reception([](void*, size_t, size_t) {}) != ReturnCode::SUCCESS) {
                        _exit(3);
                    }
                });
            }
            while (ready.load(std::memory_order_acquire) != kStarterCount) {
                std::this_thread::yield();
            }
            go.store(true, std::memory_order_release);
            for (auto& starter : starters) {
                starter.join();
            }

            const uint8_t byte = 0x42;
            if (write(sockets[1], &byte, sizeof(byte)) != static_cast<ssize_t>(sizeof(byte))) {
                _exit(4);
            }
            if (driver.stop_reception() != ReturnCode::SUCCESS) {
                _exit(5);
            }
        }
        close(sockets[1]);
        _exit(0);
    }

    EXPECT_TRUE(child_exited_successfully(child, std::chrono::milliseconds(5000)))
        << "concurrent DriverCan starts raced while assigning the reception thread";
#endif
}

TEST(DriverCanLifecycle, EmptyReceptionCallbackIsRejected) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0) {
            _exit(2);
        }

        CommandLineArgs cla{};
        TestableDriverCan driver(nullptr, cla);
        driver.adopt_socket(sockets[0]);

        const ReturnCode start_result = driver.start_reception(DriverCan::callback_t{});
        if (start_result == ReturnCode::SUCCESS) {
            const uint8_t byte = 0x42;
            if (write(sockets[1], &byte, sizeof(byte)) != static_cast<ssize_t>(sizeof(byte))) {
                _exit(3);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            _exit(4);
        }
        if (start_result != ReturnCode::INVALID_PARAM ||
            driver.close() != ReturnCode::SUCCESS) {
            _exit(5);
        }
        close(sockets[1]);
        _exit(0);
    }

    EXPECT_TRUE(child_exited_successfully(child, std::chrono::milliseconds(1000)))
        << "DriverCan accepted an empty callback that terminated its reception worker";
#endif
}

TEST(DriverCanLifecycle, ReceptionCallbackCanStopItsOwnWorker) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0) {
            _exit(2);
        }

        CommandLineArgs cla{};
        TestableDriverCan driver(nullptr, cla);
        driver.adopt_socket(sockets[0]);

        ReturnCode callback_stop_result = ReturnCode::FAIL;
        std::atomic<bool> callback_returned{false};
        if (driver.start_reception([&](void*, size_t, size_t) {
                callback_stop_result = driver.stop_reception();
                callback_returned.store(true, std::memory_order_release);
            }) != ReturnCode::SUCCESS) {
            _exit(3);
        }

        const uint8_t byte = 0x42;
        if (write(sockets[1], &byte, sizeof(byte)) != static_cast<ssize_t>(sizeof(byte))) {
            _exit(4);
        }
        const auto callback_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!callback_returned.load(std::memory_order_acquire)) {
            if (std::chrono::steady_clock::now() >= callback_deadline) {
                _exit(5);
            }
            std::this_thread::yield();
        }
        if (callback_stop_result != ReturnCode::SUCCESS || driver.close() != ReturnCode::SUCCESS) {
            _exit(6);
        }
        close(sockets[1]);
        _exit(0);
    }

    EXPECT_TRUE(child_exited_successfully(child, std::chrono::milliseconds(1000)))
        << "DriverCan callback tried to join its own reception thread";
#endif
}

TEST(DriverCanLifecycle, ReceptionCallbackCannotRestartItsOwnWorkerAfterStopping) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0) {
            _exit(2);
        }

        CommandLineArgs cla{};
        TestableDriverCan driver(nullptr, cla);
        driver.adopt_socket(sockets[0]);

        ReturnCode callback_stop_result = ReturnCode::FAIL;
        ReturnCode callback_restart_result = ReturnCode::FAIL;
        std::atomic<bool> callback_returned{false};
        if (driver.start_reception([&](void*, size_t, size_t) {
                callback_stop_result = driver.stop_reception();
                callback_restart_result = driver.start_reception([](void*, size_t, size_t) {});
                callback_returned.store(true, std::memory_order_release);
            }) != ReturnCode::SUCCESS) {
            _exit(3);
        }

        const uint8_t byte = 0x42;
        if (write(sockets[1], &byte, sizeof(byte)) != static_cast<ssize_t>(sizeof(byte))) {
            _exit(4);
        }
        const auto callback_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!callback_returned.load(std::memory_order_acquire)) {
            if (std::chrono::steady_clock::now() >= callback_deadline) {
                _exit(5);
            }
            std::this_thread::yield();
        }
        if (callback_stop_result != ReturnCode::SUCCESS ||
            callback_restart_result != ReturnCode::BUSY ||
            driver.close() != ReturnCode::SUCCESS) {
            _exit(6);
        }
        close(sockets[1]);
        _exit(0);
    }

    EXPECT_TRUE(child_exited_successfully(child, std::chrono::milliseconds(1000)))
        << "DriverCan callback overwrote its own reception thread";
#endif
}

TEST(DriverCanLifecycle, ReceptionCanRestartAfterCallbackStopsItsWorker) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0) {
            _exit(2);
        }

        CommandLineArgs cla{};
        TestableDriverCan driver(nullptr, cla);
        driver.adopt_socket(sockets[0]);

        std::atomic<bool> first_callback_returned{false};
        if (driver.start_reception([&](void*, size_t, size_t) {
                if (driver.stop_reception() != ReturnCode::SUCCESS) {
                    _exit(3);
                }
                first_callback_returned.store(true, std::memory_order_release);
            }) != ReturnCode::SUCCESS) {
            _exit(4);
        }

        const uint8_t byte = 0x42;
        if (write(sockets[1], &byte, sizeof(byte)) != static_cast<ssize_t>(sizeof(byte))) {
            _exit(5);
        }
        const auto first_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!first_callback_returned.load(std::memory_order_acquire)) {
            if (std::chrono::steady_clock::now() >= first_deadline) {
                _exit(6);
            }
            std::this_thread::yield();
        }

        std::atomic<bool> second_callback_entered{false};
        if (driver.start_reception([&](void*, size_t, size_t) {
                second_callback_entered.store(true, std::memory_order_release);
            }) != ReturnCode::SUCCESS) {
            _exit(7);
        }
        if (write(sockets[1], &byte, sizeof(byte)) != static_cast<ssize_t>(sizeof(byte))) {
            _exit(8);
        }
        const auto second_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (!second_callback_entered.load(std::memory_order_acquire)) {
            if (std::chrono::steady_clock::now() >= second_deadline) {
                _exit(9);
            }
            std::this_thread::yield();
        }
        if (driver.close() != ReturnCode::SUCCESS) {
            _exit(10);
        }
        close(sockets[1]);
        _exit(0);
    }

    EXPECT_TRUE(child_exited_successfully(child, std::chrono::milliseconds(2000)))
        << "DriverCan overwrote a stopped but still joinable reception thread";
#endif
}

TEST(DriverCanLifecycle, ReceptionCanRestartAfterHardWaitError) {
#if !defined(__linux__)
    GTEST_SKIP() << "select failure injection currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    TestableDriverCan driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    fail_next_read_select_fd.store(sockets[0], std::memory_order_release);
    read_select_failure_injected.store(false, std::memory_order_release);
    read_select_paused_after_failure.store(false, std::memory_order_release);
    release_read_select_after_failure.store(false, std::memory_order_release);
    pause_read_select_after_failure.store(true, std::memory_order_release);
    fail_next_read_select.store(true, std::memory_order_release);

    std::atomic<int> original_callback_count{0};
    ASSERT_EQ(driver.start_reception([&](void*, size_t, size_t) {
                  original_callback_count.fetch_add(1, std::memory_order_relaxed);
              }),
              ReturnCode::SUCCESS);
    ASSERT_TRUE(wait_for_flag(read_select_failure_injected, std::chrono::milliseconds(500)));

    const auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (driver.reception_running() && std::chrono::steady_clock::now() < stop_deadline) {
        std::this_thread::yield();
    }
    EXPECT_FALSE(driver.reception_running())
        << "CAN reception still reports running after a hard select error";

    std::atomic<bool> replacement_callback_entered{false};
    ASSERT_EQ(driver.start_reception([&](void*, size_t, size_t) {
                  replacement_callback_entered.store(true, std::memory_order_release);
              }),
              ReturnCode::SUCCESS);

    pause_read_select_after_failure.store(false, std::memory_order_release);
    release_read_select_after_failure.store(true, std::memory_order_release);
    const uint8_t byte = 0x42;
    ASSERT_EQ(write(sockets[1], &byte, sizeof(byte)), static_cast<ssize_t>(sizeof(byte)));
    EXPECT_TRUE(wait_for_flag(replacement_callback_entered, std::chrono::milliseconds(500)))
        << "restart kept the failed CAN worker's original callback";
    EXPECT_EQ(original_callback_count.load(std::memory_order_relaxed), 0);

    fail_next_read_select.store(false, std::memory_order_release);
    fail_next_read_select_fd.store(-1, std::memory_order_release);
    driver.close();
    close(sockets[1]);
#endif
}

TEST(DriverCanLifecycle, ReceptionCanRestartAfterHardReadError) {
#if !defined(__linux__)
    GTEST_SKIP() << "read failure injection currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    TestableDriverCan driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    fail_next_read_fd.store(sockets[0], std::memory_order_release);
    read_failure_injected.store(false, std::memory_order_release);
    read_paused_after_failure.store(false, std::memory_order_release);
    release_read_after_failure.store(false, std::memory_order_release);
    pause_read_after_failure.store(true, std::memory_order_release);
    fail_next_read.store(true, std::memory_order_release);

    std::atomic<int> original_callback_count{0};
    ASSERT_EQ(driver.start_reception([&](void*, size_t, size_t) {
                  original_callback_count.fetch_add(1, std::memory_order_relaxed);
              }),
              ReturnCode::SUCCESS);

    const uint8_t byte = 0x42;
    ASSERT_EQ(write(sockets[1], &byte, sizeof(byte)), static_cast<ssize_t>(sizeof(byte)));
    ASSERT_TRUE(wait_for_flag(read_failure_injected, std::chrono::milliseconds(500)));

    const auto stop_deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (driver.reception_running() && std::chrono::steady_clock::now() < stop_deadline) {
        std::this_thread::yield();
    }
    EXPECT_FALSE(driver.reception_running())
        << "CAN reception still reports running after a hard read error";
    if (driver.reception_running()) {
        EXPECT_TRUE(wait_for_flag(read_paused_after_failure, std::chrono::milliseconds(500)))
            << "failed CAN worker did not retry the hard read error";
    }

    std::atomic<bool> replacement_callback_entered{false};
    ASSERT_EQ(driver.start_reception([&](void*, size_t, size_t) {
                  replacement_callback_entered.store(true, std::memory_order_release);
              }),
              ReturnCode::SUCCESS);

    pause_read_after_failure.store(false, std::memory_order_release);
    release_read_after_failure.store(true, std::memory_order_release);
    EXPECT_TRUE(wait_for_flag(replacement_callback_entered, std::chrono::milliseconds(500)))
        << "restart kept the failed CAN worker's original callback";
    EXPECT_EQ(original_callback_count.load(std::memory_order_relaxed), 0);

    fail_next_read.store(false, std::memory_order_release);
    fail_next_read_fd.store(-1, std::memory_order_release);
    driver.close();
    close(sockets[1]);
#endif
}

TEST(DriverCanLifecycle, ConcurrentCloseDoesNotRetireSocketDuringSend) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    TestableDriverCan driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    delayed_send_socket.store(sockets[0], std::memory_order_release);
    delayed_send_socket_closed.store(false, std::memory_order_release);
    delayed_send_close_entered.store(false, std::memory_order_release);
    writes_after_send_socket_close.store(0, std::memory_order_relaxed);
    send_select_returning.store(false, std::memory_order_release);
    release_send_select.store(false, std::memory_order_release);
    delay_send_after_select.store(true, std::memory_order_release);

    DriverCan::can_frame_t frame{};
    ReturnCode send_result = ReturnCode::FAIL;
    std::thread sender([&] { send_result = driver.send_frame(&frame, sizeof(frame)); });
    if (!wait_for_flag(send_select_returning, std::chrono::milliseconds(500))) {
        release_send_select.store(true, std::memory_order_release);
        sender.join();
        delay_send_after_select.store(false, std::memory_order_release);
        delayed_send_socket.store(-1, std::memory_order_release);
        driver.close();
        close(sockets[1]);
        FAIL() << "send did not reach the post-select test barrier";
        return;
    }

    std::thread closer([&] { driver.close(); });
    const bool close_retired_socket_before_send_returned =
        wait_for_flag(delayed_send_close_entered, std::chrono::milliseconds(100));

    release_send_select.store(true, std::memory_order_release);
    sender.join();
    closer.join();

    EXPECT_FALSE(close_retired_socket_before_send_returned)
        << "close retired the CAN socket while send_frame still owned it";
    EXPECT_EQ(writes_after_send_socket_close.load(std::memory_order_relaxed), 0)
        << "send_frame attempted to write through a socket already retired by close";
    EXPECT_EQ(send_result, ReturnCode::SUCCESS);

    delay_send_after_select.store(false, std::memory_order_release);
    delayed_send_socket.store(-1, std::memory_order_release);
    close(sockets[1]);
#endif
}

TEST(DriverCanLifecycle, ConcurrentCloseDoesNotRetireSocketDuringRead) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    TestableDriverCan driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    DriverCan::can_frame_t expected_frame{};
    expected_frame.can_id = 0x123;
    ASSERT_EQ(write(sockets[1], &expected_frame, sizeof(expected_frame)),
              static_cast<ssize_t>(sizeof(expected_frame)));

    delayed_read_socket.store(sockets[0], std::memory_order_release);
    delayed_read_socket_closed.store(false, std::memory_order_release);
    delayed_read_close_entered.store(false, std::memory_order_release);
    reads_after_read_socket_close.store(0, std::memory_order_relaxed);
    read_select_returning.store(false, std::memory_order_release);
    release_read_select.store(false, std::memory_order_release);
    delay_read_after_select.store(true, std::memory_order_release);

    DriverCan::can_frame_t received_frame{};
    ReturnCode read_result = ReturnCode::FAIL;
    std::thread reader([&] { read_result = driver.read_frame(&received_frame, sizeof(received_frame)); });
    if (!wait_for_flag(read_select_returning, std::chrono::milliseconds(500))) {
        release_read_select.store(true, std::memory_order_release);
        reader.join();
        delay_read_after_select.store(false, std::memory_order_release);
        delayed_read_socket.store(-1, std::memory_order_release);
        driver.close();
        close(sockets[1]);
        FAIL() << "read did not reach the post-select test barrier";
        return;
    }

    std::thread closer([&] { driver.close(); });
    const bool close_retired_socket_before_read_returned =
        wait_for_flag(delayed_read_close_entered, std::chrono::milliseconds(100));

    release_read_select.store(true, std::memory_order_release);
    reader.join();
    closer.join();

    EXPECT_FALSE(close_retired_socket_before_read_returned)
        << "close retired the CAN socket while read_frame still owned it";
    EXPECT_EQ(reads_after_read_socket_close.load(std::memory_order_relaxed), 0)
        << "read_frame attempted to read through a socket already retired by close";
    EXPECT_EQ(read_result, ReturnCode::SUCCESS);
    EXPECT_EQ(received_frame.can_id, expected_frame.can_id);

    delay_read_after_select.store(false, std::memory_order_release);
    delayed_read_socket.store(-1, std::memory_order_release);
    close(sockets[1]);
#endif
}

TEST(DriverCanTransport, RejectsTruncatedFrames) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed transport assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    TestableDriverCan driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    const uint8_t truncated_frame = 0x42;
    ASSERT_EQ(write(sockets[1], &truncated_frame, sizeof(truncated_frame)),
              static_cast<ssize_t>(sizeof(truncated_frame)));
    DriverCan::can_frame_t received_frame{};

    EXPECT_EQ(driver.read_frame(&received_frame, sizeof(received_frame)), ReturnCode::FAIL);
    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
#endif
}

TEST(DriverCanTransport, RejectsInvalidBuffersBeforeIo) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed transport assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    TestableDriverCan driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    DriverCan::can_frame_t frame{};
    frame.can_id = 0x123;
    uint8_t undersized_buffer = 0x42;

    EXPECT_EQ(driver.send_frame(nullptr, sizeof(frame)), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(driver.send_frame(&undersized_buffer, sizeof(undersized_buffer)),
              ReturnCode::INVALID_PARAM);

    uint8_t peer_byte = 0;
    errno = 0;
    EXPECT_EQ(recv(sockets[1], &peer_byte, sizeof(peer_byte), MSG_DONTWAIT), -1)
        << "an invalid send emitted a truncated CAN frame";
    EXPECT_TRUE(errno == EAGAIN || errno == EWOULDBLOCK);

    ASSERT_EQ(write(sockets[1], &frame, sizeof(frame)),
              static_cast<ssize_t>(sizeof(frame)));
    EXPECT_EQ(driver.read_frame(nullptr, sizeof(frame)), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(driver.read_frame(&undersized_buffer, sizeof(undersized_buffer)),
              ReturnCode::INVALID_PARAM);

    DriverCan::can_frame_t received_frame{};
    EXPECT_EQ(driver.read_frame(&received_frame, sizeof(received_frame)),
              ReturnCode::SUCCESS)
        << "an invalid read consumed the next queued CAN frame";
    EXPECT_EQ(received_frame.can_id, frame.can_id);

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
#endif
}

TEST(DriverCanLifecycle, OpenTreatsDescriptorZeroAsAlreadyOpen) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed lifecycle assertion currently runs on Linux";
#else
    const pid_t child = fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
        int sockets[2];
        if (socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets) != 0) {
            _exit(2);
        }
        if (dup2(sockets[0], STDIN_FILENO) < 0) {
            _exit(3);
        }
        if (sockets[0] != STDIN_FILENO) {
            close(sockets[0]);
        }

        CommandLineArgs cla{};
        TestableDriverCan driver(nullptr, cla);
        driver.adopt_socket(STDIN_FILENO);
        if (driver.open(0) != ReturnCode::SUCCESS) {
            _exit(4);
        }
        if (driver.close() != ReturnCode::SUCCESS) {
            _exit(5);
        }
        close(sockets[1]);
        _exit(0);
    }

    EXPECT_TRUE(child_exited_successfully(child, std::chrono::milliseconds(1000)))
        << "DriverCan did not preserve a valid open socket stored at descriptor zero";
#endif
}
