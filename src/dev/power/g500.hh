 /**
 * @file
 * Declaration of top level class for the Gem5-power9. This class just
 * retains pointers to all its children so the children can communicate.
 *
 * Inspired from the SPARCH T1000 system.
 */

#ifndef __DEV_G500_HH__
#define __DEV_G500_HH__

#include "dev/platform.hh"
#include "params/G500.hh"

class IdeController;
class System;

class G500 : public Platform
{
    public:
    /** Pointer to the system */
    System *system;

    public:
      typedef G500Params Params;

    G500(const Params *p);
     /**
     * Cause the cpu to post a serial interrupt to the CPU.
     */
    virtual void postConsoleInt();

    /**
     * Clear a posted CPU interrupt
     */
    virtual void clearConsoleInt();

    /**
     * Cause the chipset to post a cpi interrupt to the CPU.
     */
    virtual void postPciInt(int line);

    /**
     * Clear a posted PCI->CPU interrupt
     */
    virtual void clearPciInt(int line);


    virtual Addr pciToDma(Addr pciAddr) const;

    /**
     * Calculate the configuration address given a bus/dev/func.
     */
    virtual Addr calcPciConfigAddr(int bus, int dev, int func);

    /**
     * Calculate the address for an IO location on the PCI bus.
     */
    virtual Addr calcPciIOAddr(Addr addr);

    /**
     * Calculate the address for a memory location on the PCI bus.
     */
    virtual Addr calcPciMemAddr(Addr addr);
};

#endif
