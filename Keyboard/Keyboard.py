#COM4
import serial
import keyboard
import time


ser = serial.Serial('COM4', 9600, timeout=1)  # timeout prevents hanging
time.sleep(2)  # give the port time to initialize 
print("Serial ready")
ser.write(b'X')
ser.close()

#response = ser.readline().decode().strip()  # optional: read reply from STM32

def SerialOff(event):
    ser.close()
    print("Serial closed")

def left(event):
    ser.write(b'A')  # send 'A' as a byte
    print('A')

def right(event):
    ser.write(b'D')  # send 'A' as a byte
    print('D')


#movement commands
keyboard.on_press_key("p", SerialOff)
keyboard.on_press_key("a", left)
keyboard.on_press_key("d", right)

#exit command
keyboard.wait('esc') 



