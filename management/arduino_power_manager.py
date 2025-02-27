import easy_scpi as scpi
import time

class PowerManager():
    def __init__(self, port: str) -> None:
        if port is None:
            raise ValueError('Port cannot be None.')
        self.port = f'ASRL{port}::INSTR'
        self.inst = scpi.Instrument(self.port,  read_termination='\r\n',  write_termination='\n')
        

    def power_on(self) -> None:
        self.inst.connect()
        self.inst.system.relay0(1)
        self.inst.disconnect()

    def power_off(self) -> None:
        self.inst.connect()
        self.inst.system.relay0(0)
        self.inst.disconnect()

    def restart(self) -> None:
        self.inst.connect()
        self.inst.system.do0(1)
        self.inst.system.relay0(0)
        time.sleep(2)
        self.inst.system.relay0(1)

    def bootloader(self) -> None:
        self.inst.connect()
        self.inst.system.do0(1)
        self.inst.system.relay0(0)
        time.sleep(1)
        self.inst.system.do0(0)
        self.inst.system.relay0(1)
        time.sleep(2)
        self.inst.system.do0(1)
        self.inst.disconnect()