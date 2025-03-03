import  serial
import time

class RigManager:
    def __init__(self, port, address = 0):
        self.instrument = serial.Serial(port, 9600)
        self.address = address

    def _set_power(self, output: int, state: bool):
        command = f"system:relay{output} {1 if state else 0}\n"
        print(command)
        self.instrument.write(command.encode('utf-8'))

    def _set_button(self, output: int, state: bool):
        command = f"system:do{output} {1 if state else 0}\n"
        print(command)
        self.instrument.write(command.encode('utf-8'))


    def restart(self) -> None:
        self._set_button(self.address, True)
        self._set_power(self.address, False)
        time.sleep(1)
        self._set_power(self.address, True)

    def bootloader(self) -> None:
        self._set_power(self.address, False)
        time.sleep(1)
        self._set_button(self.address, False)
        self._set_power(self.address, True)
        time.sleep(2.5)
        self._set_button(self.address, True)

    def close(self):
        self.instrument.close()

# Test code
if __name__ == "__main__":
    import time

    rig_manager = RigManager("/dev/ttyACM1")

    rig_manager.restart()