/*
 * Copyright (c) 2009 The Regents of The University of Michigan
 * Copyright (c) 2009 The University of Edinburgh
 * All rights reserved.
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
 *          Timothy M. Jones
 */

#ifndef __ARCH_POWER_ISA_HH__
#define __ARCH_POWER_ISA_HH__

#include "arch/power/registers.hh"
#include "arch/power/types.hh"
#include "base/logging.hh"
#include "cpu/reg_class.hh"
#include "debug/MiscRegs.hh"
#include "sim/sim_object.hh"

struct PowerISAParams;
class ThreadContext;
class Checkpoint;
class EventManager;

namespace PowerISA
{

class ISA : public SimObject
{
  protected:
    RegVal dummy;
    RegVal miscRegs[NumMiscRegs];

  public:
    typedef PowerISAParams Params;

    void
    clear()
    {
    }

    RegVal
    readMiscRegNoEffect(int miscReg) const
    {
        assert(miscReg < NumMiscRegs);
        int flatIndex = flattenMiscIndex(miscReg);
        auto regVal = miscRegs[flatIndex];
        auto regName = miscRegName[flatIndex];
        DPRINTF(MiscRegs, "Reading misc reg %d (%s) as %#x.\n", miscReg,
                regName, regVal);
        return regVal;
    }

    RegVal
    readMiscReg(int miscReg, ThreadContext *tc)
    {
        return readMiscRegNoEffect(miscReg);
    }

    void
    setMiscRegNoEffect(int miscReg, RegVal regVal)
    {
        assert(miscReg < NumMiscRegs);
        int flatIndex = flattenMiscIndex(miscReg);
        auto regName = miscRegName[flatIndex];
        DPRINTF(MiscRegs, "Setting misc reg %d (%s) to %#x.\n", miscReg,
                regName, regVal);
        miscRegs[flatIndex] = regVal;
    }

    void
    setMiscReg(int miscReg, RegVal regVal, ThreadContext *tc)
    {
        return setMiscRegNoEffect(miscReg, regVal);
    }

    RegId
    flattenRegId(const RegId& regId) const
    {
        switch (regId.classValue()) {
            case IntRegClass:
                return RegId(IntRegClass, flattenIntIndex(regId.index()));
            case FloatRegClass:
                return RegId(FloatRegClass, flattenFloatIndex(regId.index()));
            case VecRegClass:
                return RegId(VecRegClass, flattenVecIndex(regId.index()));
            case VecElemClass:
                return RegId(VecElemClass, flattenVecElemIndex(regId.index()),
                             regId.elemIndex());
            case VecPredRegClass:
                return RegId(VecPredRegClass,
                             flattenVecPredIndex(regId.index()));
            case CCRegClass:
                return RegId(CCRegClass, flattenCCIndex(regId.index()));
            case MiscRegClass:
                return RegId(MiscRegClass, flattenMiscIndex(regId.index()));
        }

        return RegId();
    }

    int
    flattenIntIndex(int reg) const
    {
        return reg;
    }

    int
    flattenFloatIndex(int reg) const
    {
        return reg;
    }

    int
    flattenVecIndex(int reg) const
    {
        return reg;
    }

    int
    flattenVecElemIndex(int reg) const
    {
        return reg;
    }

    int
    flattenVecPredIndex(int reg) const
    {
        return reg;
    }

    // dummy
    int
    flattenCCIndex(int reg) const
    {
        return reg;
    }

    int
    flattenMiscIndex(int reg) const
    {
        return reg;
    }

    void startup(ThreadContext *tc) {}

    /// Explicitly import the otherwise hidden startup
    using SimObject::startup;

    const Params *params() const;

    ISA(Params *p);
};

} // namespace PowerISA

#endif // __ARCH_POWER_ISA_HH__
