/*-------------------------------------------------------------------------
* Copyright (c) 2025 Ainekko, Co.
* SPDX-License-Identifier: Apache-2.0
*-------------------------------------------------------------------------*/

#include "emu_defines.h"
#include "devices/sysregs_er.h"
#include "system.h"
#include "memory/sysreg_region.h"
#include "memory/dense_region.h"

namespace bemu {

void MainMemory::reset()
{
    regions[erbreg_idx].reset(new SysregsEr<region_bases[erbreg_idx]>());
    regions[bootrom_idx].reset(new DenseRegion<region_bases[bootrom_idx], region_sizes[bootrom_idx], false>());
    regions[sram_idx].reset(new DenseRegion<region_bases[sram_idx], region_sizes[sram_idx]>());
    regions[dram_idx].reset(new DenseRegion<region_bases[dram_idx], region_sizes[dram_idx]>());
    regions[sysreg_idx].reset(new SysregRegion<region_bases[sysreg_idx], region_sizes[sysreg_idx]>());
}

void MainMemory::wdt_clock_tick(const Agent& agent, uint64_t cycle)
{
    auto ptr = dynamic_cast<SysregsEr<region_bases[erbreg_idx]>*>(regions[erbreg_idx].get());
    ptr->wdt_clock_tick(agent, cycle);
}

} // namespace bemu
