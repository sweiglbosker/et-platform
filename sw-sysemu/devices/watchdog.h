/*-------------------------------------------------------------------------
* Copyright (c) 2025 Ainekko, Co.
* SPDX-License-Identifier: Apache-2.0
*-------------------------------------------------------------------------*/

#ifndef BEMU_WATCHDOG_H
#define BEMU_WATCHDOG_H

#include <cstdint>
#include "agent.h"

namespace bemu {

// Forward declaration
class System;

// Watchdog timeout handler function type
// Parameter: const Agent& agent who triggered the tick
using WatchdogTimeoutHandler = void (*)(const Agent& agent);

// Watchdog clock configuration
// Base system clock is 1GHz (1 cycle = 1ns)
// Watchdog clock = System clock / ClockDivider
// Default: divider=4 gives 250MHz watchdog clock
template<uint32_t ClockDivider = 4>
class Watchdog {
public:
    Watchdog() = default;
    
    void set_timeout_handler(WatchdogTimeoutHandler handler) {
        timeout_handler = handler;
    }
    
    void set_count_from(uint32_t count_value) {
        count_from = count_value;
    }
    
    // Reset/kick the watchdog - reload counter from stored count
    void kick() {
        current_value = count_from;
    }
    
    void handle_zero(const Agent& agent) {
        if (timeout_handler) {
            timeout_handler(agent);
        }
    }
    
    void set_enabled(bool enabled) { this->enabled = enabled; }
    bool is_enabled() const { return enabled; }
    
    void clock_tick(const Agent& agent, uint64_t cycle) {
        // Apply clock divider: only tick every ClockDivider cycles
        if ((cycle % ClockDivider) != 0) {
            return;
        }

        if (!enabled) {
            return;
        }

        if (current_value > 0) {
            current_value--;

            if (current_value == 0) {
                handle_zero(agent);
            }
        }
    }
    
    uint32_t get_current_value() const { return current_value; }
    
    uint32_t get_count_from() const { return count_from; }

private:
    uint32_t count_from = 0xFFFF;      // Value to reload counter from
    uint32_t current_value = 0xFFFF;   // Current countdown value
    bool enabled = false;
    WatchdogTimeoutHandler timeout_handler = nullptr;  // Function to call on timeout
};

} // namespace bemu

#endif // BEMU_WATCHDOG_H
