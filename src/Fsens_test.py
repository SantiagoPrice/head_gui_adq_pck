#! /usr/bin/env python3
import serial
import threading

timer_flag=False

def callback():
    global timer_flag
    timer_flag=True

timer = threading.Timer(10.0, callback)

ser = serial.Serial('/dev/ttyUSB1', baudrate=921600, bytesize=8, timeout=None,parity="N",stopbits=1, rtscts=1)
ser.reset_input_buffer()
# Set up
#A2
ser.write(bytes.fromhex('41'))
ser.write(bytes.fromhex('32'))

#F1
ser.write(bytes.fromhex('46'))
ser.write(bytes.fromhex('31'))

#L0
ser.write(bytes.fromhex('4C'))
ser.write(bytes.fromhex('30'))
#I
ser.write(bytes.fromhex('49'))
#HC
ser.write(bytes.fromhex('48'))
ser.write(bytes.fromhex('43'))

# Adquisition 


#R
ser.write(bytes.fromhex('52'))
ser.write(bytes.fromhex('53'))

timer.start()
# First read
FXavg=0
FYavg=0
FZavg=0
i=0

#print(f"FX is {float((FXavg-16384))/240:.3f}")
while True:
    while True:
        st=ser.read(1)
        if st==b'\n':
            break

    seq=ser.read(1)
    FXr=ser.read(4)
    FYr=ser.read(4)
    FZr=ser.read(4)
    

    i+=1
    FX=int(str(FXr)[2:-1],16)
    FXavg+=(FX-16384)

    FY=int(str(FYr)[2:-1],16)
    FYavg+=(FY-16384)

    FZ=int(str(FZr)[2:-1],16)
    FZavg+=(FZ-16384)


    #print(f"seq is {int.from_bytes(seq, byteorder='little')}")
    if i>1500:
        i=0
        #print(f"FZ is {FZavg}")
        #print(f"FZ is {float(FZ)/240:.3f}")

        print(f"FX is {float(FXavg)/1500/240:.3f}")
        print(f"FY is {float(FYavg)/1500/240:.3f}") 
        print(f"FZ is {float(FZavg)/1500/240:.3f}")
        FXavg=0
        FYavg=0
        FZavg=0
        #print(f"FY is {int.from_bytes(bytes.fromhex(FY), byteorder='big')}")
        #print(f"FZ is {int.from_bytes(FZ, byteorder='big')}")
        #print(f"Status is is {st}")
    if timer_flag:
        print("end of the communication")
        ser.write(bytes.fromhex('45'))
        break

