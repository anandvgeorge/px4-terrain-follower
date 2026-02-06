#pragma once

#include <semaphore.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <string>
#include <chrono>
#include <cstring>
#include <iostream>
#include "shared_memory.h"
#include "types/drone_command.h"

namespace terrain_follower {
namespace ipc {

/**
 * @brief Command channel using shared memory + semaphores for synchronous command/response pattern
 * 
 * This provides a ROS-service-like interface for sending commands to px4_mavlink_interface.
 * Server (px4_mavlink_interface) blocks on wait_for_command() until a command arrives.
 * Client (ControlNode, OffboardMode) blocks on send_command() until response is received.
 */
class CommandChannel {
public:
    explicit CommandChannel(const std::string& name) 
        : shm_(name),
          cmd_sem_name_("/cmd_" + name),
          response_sem_name_("/resp_" + name) {
    }
    
    ~CommandChannel() {
        close();
    }
    
    // Prevent copying
    CommandChannel(const CommandChannel&) = delete;
    CommandChannel& operator=(const CommandChannel&) = delete;
    
    /**
     * @brief Create command channel (server side)
     * Creates shared memory and semaphores. Call this from px4_mavlink_interface.
     */
    bool create() {
        // Create shared memory
        if (!shm_.create()) {
            std::cerr << "[CommandChannel] Failed to create shared memory" << std::endl;
            return false;
        }
        
        // Clean up any existing semaphores from previous crash
        sem_unlink(cmd_sem_name_.c_str());
        sem_unlink(response_sem_name_.c_str());
        
        // Create semaphores (initial value = 0, meaning no commands/responses pending)
        cmd_sem_ = sem_open(cmd_sem_name_.c_str(), O_CREAT | O_EXCL, 0644, 0);
        if (cmd_sem_ == SEM_FAILED) {
            std::cerr << "[CommandChannel] Failed to create command semaphore: " 
                      << strerror(errno) << std::endl;
            return false;
        }
        
        response_sem_ = sem_open(response_sem_name_.c_str(), O_CREAT | O_EXCL, 0644, 0);
        if (response_sem_ == SEM_FAILED) {
            std::cerr << "[CommandChannel] Failed to create response semaphore: " 
                      << strerror(errno) << std::endl;
            sem_close(cmd_sem_);
            sem_unlink(cmd_sem_name_.c_str());
            return false;
        }
        
        std::cout << "[CommandChannel] Created: " << cmd_sem_name_ << ", " << response_sem_name_ << std::endl;
        return true;
    }
    
    /**
     * @brief Open existing command channel (client side)
     * Opens shared memory and semaphores. Call this from ControlNode/OffboardMode.
     */
    bool open() {
        // Open shared memory
        if (!shm_.open()) {
            std::cerr << "[CommandChannel] Failed to open shared memory" << std::endl;
            return false;
        }
        
        // Open existing semaphores
        cmd_sem_ = sem_open(cmd_sem_name_.c_str(), 0);
        if (cmd_sem_ == SEM_FAILED) {
            std::cerr << "[CommandChannel] Failed to open command semaphore: " 
                      << strerror(errno) << std::endl;
            return false;
        }
        
        response_sem_ = sem_open(response_sem_name_.c_str(), 0);
        if (response_sem_ == SEM_FAILED) {
            std::cerr << "[CommandChannel] Failed to open response semaphore: " 
                      << strerror(errno) << std::endl;
            sem_close(cmd_sem_);
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief Wait for command (server side - blocking with timeout)
     * Blocks until a command arrives or timeout. Returns false on error/timeout.
     * Uses short timeout to allow checking exit conditions.
     */
    bool wait_for_command(DroneCommand& cmd) {
        // Use timedwait with short timeout to be responsive to signals
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 1;  // 1 second timeout
        
        int ret = sem_timedwait(cmd_sem_, &ts);
        if (ret != 0) {
            if (errno == EINTR || errno == ETIMEDOUT) {
                // Interrupted by signal or timeout - not an error
                return false;
            }
            std::cerr << "[CommandChannel] sem_timedwait failed: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Read command from shared memory
        if (!shm_.read(cmd)) {
            std::cerr << "[CommandChannel] Failed to read command" << std::endl;
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief Send response (server side - non-blocking)
     * Writes response to shared memory and signals client.
     */
    void send_response(const DroneCommand& response) {
        // Write response to shared memory
        if (!shm_.write(response)) {
            std::cerr << "[CommandChannel] Failed to write response" << std::endl;
            return;
        }
        
        // Signal response available
        sem_post(response_sem_);
    }
    
    /**
     * @brief Send command and wait for response (client side - blocking with timeout)
     * @param cmd Command to send
     * @param timeout_ms Timeout in milliseconds (default 5000ms)
     * @return true if command succeeded, false if failed or timeout
     */
    bool send_command(const DroneCommand& cmd, int timeout_ms = 5000) {
        // Write command to shared memory
        if (!shm_.write(cmd)) {
            std::cerr << "[CommandChannel] Failed to write command" << std::endl;
            return false;
        }
        
        // Signal server that command is available
        if (sem_post(cmd_sem_) != 0) {
            std::cerr << "[CommandChannel] Failed to signal command: " << strerror(errno) << std::endl;
            return false;
        }
        
        // Wait for response with timeout
        struct timespec ts;
        if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
            std::cerr << "[CommandChannel] clock_gettime failed" << std::endl;
            return false;
        }
        
        // Add timeout
        ts.tv_sec += timeout_ms / 1000;
        ts.tv_nsec += (timeout_ms % 1000) * 1000000L;
        
        // Handle nanosecond overflow
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000L;
        }
        
        // Wait for response semaphore with timeout
        if (sem_timedwait(response_sem_, &ts) != 0) {
            if (errno == ETIMEDOUT) {
                std::cerr << "[CommandChannel] Command timeout after " << timeout_ms << "ms" << std::endl;
            } else if (errno == EINTR) {
                std::cerr << "[CommandChannel] Wait interrupted" << std::endl;
            } else {
                std::cerr << "[CommandChannel] sem_timedwait failed: " << strerror(errno) << std::endl;
            }
            return false;
        }
        
        // Read response from shared memory
        DroneCommand response;
        if (!shm_.read(response)) {
            std::cerr << "[CommandChannel] Failed to read response" << std::endl;
            return false;
        }
        
        // Check response status
        return response.status == CommandStatus::SUCCESS;
    }
    
    /**
     * @brief Close command channel
     * Closes semaphores and shared memory. If created by this process, also unlinks them.
     */
    void close() {
        if (cmd_sem_ != SEM_FAILED) {
            sem_close(cmd_sem_);
            cmd_sem_ = SEM_FAILED;
        }
        
        if (response_sem_ != SEM_FAILED) {
            sem_close(response_sem_);
            response_sem_ = SEM_FAILED;
        }
        
        shm_.close();
    }
    
    /**
     * @brief Cleanup semaphores (call on server shutdown)
     * Unlinks semaphores from filesystem. Only call this from server side.
     */
    void cleanup() {
        sem_unlink(cmd_sem_name_.c_str());
        sem_unlink(response_sem_name_.c_str());
    }

private:
    SharedMemory<DroneCommand> shm_;
    sem_t* cmd_sem_ = SEM_FAILED;       // Signals: new command available
    sem_t* response_sem_ = SEM_FAILED;  // Signals: response ready
    std::string cmd_sem_name_;
    std::string response_sem_name_;
};

} // namespace ipc
} // namespace terrain_follower
