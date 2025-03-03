import  serial
import time

class RigManager:
    def __init__(self, port, address = 0):
        self.instrument = serial.Serial(port, 9600)
        self.address = address

    def _set_power(self, output: int, state: bool):
        command = f"system:relay{output} {1 if state else 0}\n"
        self.instrument.write(command.encode('utf-8'))

    def _set_button(self, output: int, state: bool):
        command = f"system:do{output} {1 if state else 0}\n"
        self.instrument.write(command.encode('utf-8'))


    def restart(self,address=0) -> None:
        self._set_button(address, True)
        self._set_power(address, False)
        time.sleep(1)
        self._set_power(address, True)

    def bootloader(self,address=0) -> None:
        self._set_power(address, False)
        time.sleep(1)
        self._set_button(address, False)
        self._set_power(address, True)
        time.sleep(2.5)
        self._set_button(address, True)


# Test code
if __name__ == "__main__":
    import time

    rig_manager = RigManager("/dev/tty.usbmodemF412FA6372CC2")

    rig_manager.restart()