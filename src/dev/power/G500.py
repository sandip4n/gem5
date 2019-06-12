from m5.params import *
from m5.proxy import *
from Device import BasicPioDevice, PioDevice, IsaFake, BadAddr
from Platform import Platform
from Terminal import Terminal
from Uart import Uart8250

class G500(Platform):
    type = 'G500'
    cxx_header = "dev/power/g500.hh"
    system = Param.System(Parent.any, "system")
    pterm = Terminal()
    puart0 = Uart8250(pio_addr=0xFFFF4505)

    def attachIO(self,bus):
        self.puart0.device = self.pterm
        self.puart0.pio = bus.master
