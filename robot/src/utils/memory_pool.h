#ifndef MEMORY_POOL_H
#define MEMORY_POOL_H

#include "../config.h"

/**
 * @brief Simple memory pool for pre-allocated objects to avoid dynamic allocation
 * 
 * This class provides a simple memory pool implementation to pre-allocate
 * commonly used objects and avoid dynamic memory allocation during runtime.
 */
template<typename T, size_t PoolSize>
class MemoryPool {
private:
    alignas(T) char pool[PoolSize * sizeof(T)];
    bool used[PoolSize];
    size_t next_free;

public:
    MemoryPool() : next_free(0) {
        for (size_t i = 0; i < PoolSize; ++i) {
            used[i] = false;
        }
    }

    /**
     * @brief Allocate an object from the pool
     * @return Pointer to allocated object, or nullptr if pool is full
     */
    T* allocate() {
        // Fast path: check next_free first
        if (next_free < PoolSize && !used[next_free]) {
            used[next_free] = true;
            T* result = reinterpret_cast<T*>(&pool[next_free * sizeof(T)]);
            ++next_free;
            return result;
        }

        // Slow path: search for free slot
        for (size_t i = 0; i < PoolSize; ++i) {
            if (!used[i]) {
                used[i] = true;
                next_free = i + 1;
                return reinterpret_cast<T*>(&pool[i * sizeof(T)]);
            }
        }

        return nullptr; // Pool is full
    }

    /**
     * @brief Deallocate an object back to the pool
     * @param ptr Pointer to object to deallocate
     */
    void deallocate(T* ptr) {
        if (!ptr) return;

        char* char_ptr = reinterpret_cast<char*>(ptr);
        if (char_ptr >= pool && char_ptr < pool + sizeof(pool)) {
            size_t index = (char_ptr - pool) / sizeof(T);
            if (index < PoolSize) {
                used[index] = false;
                if (index < next_free) {
                    next_free = index;
                }
            }
        }
    }

    /**
     * @brief Get number of available slots in the pool
     * @return Number of free slots
     */
    size_t available() const {
        size_t count = 0;
        for (size_t i = 0; i < PoolSize; ++i) {
            if (!used[i]) ++count;
        }
        return count;
    }

    /**
     * @brief Check if pool is full
     * @return true if no free slots available
     */
    inline bool is_full() const {
        return available() == 0;
    }
};

// Pre-defined memory pools for common objects
extern MemoryPool<int, config::constants::PWM_CALCULATION_POOL_SIZE> pwm_calculation_pool;

#endif // MEMORY_POOL_H
