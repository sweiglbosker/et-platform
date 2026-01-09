/*-------------------------------------------------------------------------
* Copyright (c) 2025 Ainekko, Co.
* SPDX-License-Identifier: Apache-2.0
*-------------------------------------------------------------------------*/

#ifndef BEMU_PLIC_ER_H
#define BEMU_PLIC_ER_H

#include "plic_dev.h"

namespace bemu {

//
// Erbium PLIC Implementation.
//
template <unsigned long long Base, size_t N>
struct ER_PLIC : public PLIC<Base, N, 32, 2>
{
    static void Target_Machine_external_interrupt(System* system, bool raise) {
        if (raise) {
            system->raise_machine_external_interrupt(0);
        } else {
            system->clear_machine_external_interrupt(0);
        }
    }

    static void Target_Supervisor_external_interrupt(System* system, bool raise) {
        if (raise) {
            system->raise_supervisor_external_interrupt(0);
        } else {
            system->clear_supervisor_external_interrupt(0);
        }
    }

    const std::vector<PLIC_Interrupt_Target> &get_target_list() const override {
        static const std::vector<PLIC_Interrupt_Target> targets = {
            {0, 0, Target_Machine_external_interrupt},
            {1, 1, Target_Supervisor_external_interrupt},
        };
        return targets;
    }
};

} // namespace bemu

#endif // BEMU_PLIC_ERBIUM_H
