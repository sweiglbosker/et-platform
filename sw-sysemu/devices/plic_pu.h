/*-------------------------------------------------------------------------
* Copyright (c) 2025 Ainekko, Co.
* SPDX-License-Identifier: Apache-2.0
*-------------------------------------------------------------------------*/

#ifndef BEMU_PLIC_PU_H
#define BEMU_PLIC_PU_H

#include "plic_dev.h"


namespace bemu {

//
// Peripheral PLIC Implementation.
//
template <unsigned long long Base, size_t N>
struct PU_PLIC : public PLIC<Base, N, 41, 12>
{
    static void Target_Minion_Machine_external_interrupt(System* system, bool raise) {
        for (int i = 0; i < EMU_NUM_MINION_SHIRES; i++) {
            if (raise) {
                system->raise_machine_external_interrupt(i);
            } else {
                system->clear_machine_external_interrupt(i);
            }
        }
    }

    static void Target_Minion_Supervisor_external_interrupt(System* system, bool raise) {
        for (int i = 0; i < EMU_NUM_MINION_SHIRES; i++) {
            if (raise) {
                system->raise_supervisor_external_interrupt(i);
            } else {
                system->clear_supervisor_external_interrupt(i);
            }
        }
    }

    const std::vector<PLIC_Interrupt_Target> &get_target_list() const {
        static const std::vector<PLIC_Interrupt_Target> targets = {
            {10, 0x21, Target_Minion_Machine_external_interrupt},
            {11, 0x20, Target_Minion_Supervisor_external_interrupt},
        };
        return targets;
    }
};

} // namespace bemu

#endif // BEMU_PLIC_H
