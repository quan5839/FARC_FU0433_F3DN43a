#include "memory_pool.h"

// Initialize global memory pools
MemoryPool<int, config::constants::PWM_CALCULATION_POOL_SIZE> pwm_calculation_pool;
