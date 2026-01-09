/*-------------------------------------------------------------------------
* Copyright (c) 2025 Ainekko, Co.
* SPDX-License-Identifier: Apache-2.0
*-------------------------------------------------------------------------*/

#ifndef BEMU_PLIC_SP_H
#define BEMU_PLIC_SP_H

#include "plic_dev.h"

namespace bemu {

//
// ETSOC-1 Service Processor PLIC
//
template <unsigned long long Base, size_t N>
struct SP_PLIC : public PLIC<Base, N, 148, 2>
{
    static void Target_SP_Machine_external_interrupt(System* system, bool raise) {
        if (raise) {
            system->raise_machine_external_interrupt(IO_SHIRE_ID);
        } else {
            system->clear_machine_external_interrupt(IO_SHIRE_ID);
        }
    }

    static void Target_SP_Supervisor_external_interrupt(System* system, bool raise) {
        if (raise) {
            system->raise_supervisor_external_interrupt(IO_SHIRE_ID);
        } else {
            system->clear_supervisor_external_interrupt(IO_SHIRE_ID);
        }
    }

    const std::vector<PLIC_Interrupt_Target> &get_target_list() const {
        static const std::vector<PLIC_Interrupt_Target> targets = {
            {0, 0, Target_SP_Machine_external_interrupt},
            {1, 1, Target_SP_Supervisor_external_interrupt},
        };
        return targets;
    }
};

} // namespace bemu

#endif // BEMU_PLIC_H
