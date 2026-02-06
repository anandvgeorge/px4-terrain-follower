#pragma once

#include <string>
#include <memory>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>

namespace terrain_follower {
namespace ipc {

template<typename T>
class SharedMemory {
public:
    explicit SharedMemory(const std::string& name);
    ~SharedMemory();
    
    // Create shared memory (writer)
    bool create();
    
    // Open existing shared memory (reader)
    bool open();
    
    // Write data
    bool write(const T& data);
    
    // Read data
    bool read(T& data);
    
    // Close and cleanup
    void close();
    
    bool is_open() const { return ptr_ != nullptr; }

private:
    std::string name_;
    std::string sem_name_;
    int fd_;
    T* ptr_;
    sem_t* sem_;
    bool is_creator_;
    
    static constexpr size_t MEMORY_SIZE = sizeof(T);
};

// Implementation (template class)
template<typename T>
SharedMemory<T>::SharedMemory(const std::string& name)
    : name_("/" + name)
    , sem_name_("/" + name + "_sem")
    , fd_(-1)
    , ptr_(nullptr)
    , sem_(SEM_FAILED)
    , is_creator_(false) {
}

template<typename T>
SharedMemory<T>::~SharedMemory() {
    close();
}

template<typename T>
bool SharedMemory<T>::create() {
    // Create shared memory
    fd_ = shm_open(name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd_ == -1) {
        return false;
    }
    
    // Set size
    if (ftruncate(fd_, MEMORY_SIZE) == -1) {
        ::close(fd_);
        return false;
    }
    
    // Map to memory
    ptr_ = static_cast<T*>(mmap(nullptr, MEMORY_SIZE, 
                                 PROT_READ | PROT_WRITE, 
                                 MAP_SHARED, fd_, 0));
    if (ptr_ == MAP_FAILED) {
        ::close(fd_);
        return false;
    }
    
    // Create semaphore for synchronization
    sem_ = sem_open(sem_name_.c_str(), O_CREAT, 0666, 1);
    if (sem_ == SEM_FAILED) {
        munmap(ptr_, MEMORY_SIZE);
        ::close(fd_);
        return false;
    }
    
    is_creator_ = true;
    return true;
}

template<typename T>
bool SharedMemory<T>::open() {
    // Open existing shared memory
    fd_ = shm_open(name_.c_str(), O_RDWR, 0666);
    if (fd_ == -1) {
        return false;
    }
    
    // Map to memory
    ptr_ = static_cast<T*>(mmap(nullptr, MEMORY_SIZE, 
                                 PROT_READ | PROT_WRITE, 
                                 MAP_SHARED, fd_, 0));
    if (ptr_ == MAP_FAILED) {
        ::close(fd_);
        return false;
    }
    
    // Open semaphore
    sem_ = sem_open(sem_name_.c_str(), 0);
    if (sem_ == SEM_FAILED) {
        munmap(ptr_, MEMORY_SIZE);
        ::close(fd_);
        return false;
    }
    
    is_creator_ = false;
    return true;
}

template<typename T>
bool SharedMemory<T>::write(const T& data) {
    if (!ptr_ || sem_ == SEM_FAILED) {
        return false;
    }
    
    // Lock
    sem_wait(sem_);
    
    // Write data
    *ptr_ = data;
    
    // Unlock
    sem_post(sem_);
    
    return true;
}

template<typename T>
bool SharedMemory<T>::read(T& data) {
    if (!ptr_ || sem_ == SEM_FAILED) {
        return false;
    }
    
    // Lock
    sem_wait(sem_);
    
    // Read data
    data = *ptr_;
    
    // Unlock
    sem_post(sem_);
    
    return true;
}

template<typename T>
void SharedMemory<T>::close() {
    if (ptr_ != nullptr && ptr_ != MAP_FAILED) {
        munmap(ptr_, MEMORY_SIZE);
        ptr_ = nullptr;
    }
    
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
    
    if (sem_ != SEM_FAILED) {
        sem_close(sem_);
        if (is_creator_) {
            sem_unlink(sem_name_.c_str());
        }
        sem_ = SEM_FAILED;
    }
    
    if (is_creator_) {
        shm_unlink(name_.c_str());
    }
}

} // namespace ipc
} // namespace terrain_follower
