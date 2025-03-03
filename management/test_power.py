import serial


class RigManager:
    def __init__(self, port):
        self.instrument = serial.Serial(port, 9600)

    def _set_power(self, output: int, state: bool):
        command = f"system:relay{output} {1 if state else 0}\n"
        print(command)
        self.instrument.write(command.encode('utf-8'))

    def _set_button(self, output: int, state: bool):
        command = f"system:do{output} {0 if state else 1}\n"
        print(command)
        self.instrument.write(command.encode('utf-8'))

# Test code
if __name__ == "__main__":
    import time

    rig_manager = RigManager("/dev/ttyACM1")

    # Reset Crazyflie to bootloader mode
    rig_manager.set_power(0, False)
    time.sleep(1)
    rig_manager.set_button(0, True)
    rig_manager.set_power(0, True)
    time.sleep(2.5)
    rig_manager.set_button(0, False)