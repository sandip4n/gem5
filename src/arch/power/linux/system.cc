/*
 * Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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

#include "arch/power/linux/system.hh"

#include "arch/vtophys.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "debug/Loader.hh"
#include "params/LinuxPowerSystem.hh"
#include "sim/stat_control.hh"

using namespace PowerISA;

LinuxPowerSystem::LinuxPowerSystem(Params *p)
    : PowerSystem(p)
{
}

LinuxPowerSystem::~LinuxPowerSystem()
{
}

void
LinuxPowerSystem::initState()
{
    PowerSystem::initState();

    if (params()->early_kernel_symbols) {
        kernel->loadGlobalSymbols(kernelSymtab, 0, 0, loadAddrMask);
        kernel->loadGlobalSymbols(debugSymbolTable, 0, 0, loadAddrMask);
    }

    // Setup boot data structure
    Addr addr = 0;
    // Check if the kernel image has a symbol that tells us it supports
    // device trees.
    bool kernel_has_fdt_support =
        kernelSymtab->findAddress("unflatten_device_tree", addr);
    bool dtb_file_specified = params()->dtb_filename != "";

    if (kernel_has_fdt_support && dtb_file_specified) {
        // Kernel supports flattened device tree and dtb file specified.
        // Using Device Tree Blob to describe system configuration.
        inform("Loading DTB file: %s at address %#x\n", params()->dtb_filename,
                0x1800000 +loadAddrOffset);

        ObjectFile *dtb_file = createObjectFile(params()->dtb_filename, true);
        if (!dtb_file) {
            fatal("couldn't load DTB file: %s\n", params()->dtb_filename);
        }

        DtbObject *_dtb_file = dynamic_cast<DtbObject*>(dtb_file);

        if (_dtb_file) {
            std::cout<<params()->boot_osflags.c_str()<<std::endl;
            if (!_dtb_file->addBootCmdLine(params()->boot_osflags.c_str(),
                                           params()->boot_osflags.size())) {
                warn("couldn't append bootargs to DTB file: %s\n",
                     params()->dtb_filename);
            }
        } else {
            warn("dtb_file cast failed; couldn't append bootargs "
                 "to DTB file: %s\n", params()->dtb_filename);
        }

        dtb_file->setTextBase(0x1800000 +loadAddrOffset);
        dtb_file->loadSections(physProxy);
        delete dtb_file;
    }
}

LinuxPowerSystem *
LinuxPowerSystemParams::create()
{
    return new LinuxPowerSystem(this);
}
