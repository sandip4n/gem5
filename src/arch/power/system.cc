/*
 * Copyright (c) 2007 The Hewlett-Packard Development Company
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Gabe Black
 */

#include "arch/power/system.hh"

#include "arch/power/isa_traits.hh"
#include "arch/power/miscregs.hh"
#include "arch/power/registers.hh"
#include "base/loader/object_file.hh"
#include "cpu/thread_context.hh"
#include "params/PowerSystem.hh"

using namespace PowerISA;
PowerSystem::PowerSystem(Params *p) :
    System(p)
{
}

PowerSystem::~PowerSystem()
{
}

PowerSystem *
PowerSystemParams::create()
{
    return new PowerSystem(this);
}

void
PowerSystem::initState()
{
    System::initState();

    for (int i = 0; i < threadContexts.size(); i++) {
        ThreadContext *tc = threadContexts[i];
        // Entry Point of Skiboot
        // Using known address instead of tc->getSystemPtr()->kernelEntry
        tc->pcState(0x10);

        // Set Decrementer and Timebase
        tc->setIntReg(INTREG_DEC, 0x0);
        tc->setIntReg(INTREG_TB, 0x0);

        // Set Processor Version corresponding to POWER9
        tc->setIntReg(INTREG_PVR, 0x004e0200);

        // 64-bit, big-endian hypervisor real mode
        // SF and HV are set, IR and DR bits are not
        tc->setIntReg(INTREG_MSR, 0x9000000000000000);

        // Device Tree is loaded at 0x1800000 and this address
        // is passed as an argument to skiboot
        tc->setIntReg(ArgumentReg0, 0x1800000);

        // Processor Identification
        tc->setIntReg(INTREG_PIR, i);
    }
}
