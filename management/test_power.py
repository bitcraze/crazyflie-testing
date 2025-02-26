import easy_scpi as scpi
import time

inst = scpi.Instrument('ASRL/dev/ttyACM0::INSTR',  read_termination='\r\n',  write_termination='\n')

inst.connect()

inst.system.relay0(0)
time.sleep(2)

inst.system.do0(0)
inst.system.relay0(1)

time.sleep(2.5)

inst.system.do0(1)


time.sleep(20)
inst.system.relay0(0)

print(inst.system.relay0())