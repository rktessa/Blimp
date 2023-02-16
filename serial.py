import Blimp.serial as serial
import time

mesg = {}

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


#SET OUTPUT MESSAGE
hser = serial.Serial( '/dev/serial0', 921600, timeout = 0)
rl = ReadLine(hser)

try:
    while True:
        mesg = rl.readline().decode("utf-8")
        print(mesg)


except KeyboardInterrupt:
    hser.close()
    print ('Serial port closed')

