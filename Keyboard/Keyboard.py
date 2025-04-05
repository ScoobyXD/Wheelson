#COM4
import serial
import keyboard
import time


ser = serial.Serial('COM4', 9600, timeout=1)  # timeout prevents hanging
time.sleep(2)  # give the port time to initialize 
print("Serial ready")

def up(event):
    ser.write(b'w')  #send 'a' as a byte
    print('w')

def down(event):
    ser.write(b's')  #send 'd' as a byte
    print('s')

def left(event):
    ser.write(b'a')  #send 'a' as a byte
    print('a')

def right(event):
    ser.write(b'd')  #send 'd' as a byte
    print('d')

def fire(event):
    ser.write(b' ')  #send ' ' as a byte
    print(' ')

def udpause(event):
    ser.write(b'y') #send 'x' as a byte
    print('y')

def lrpause(event):
    ser.write(b'x') #send 'x' as a byte
    print('x')

def firepause(event):
    ser.write(b'z') #send 'x' as a byte
    print('z')

def SerialOff(event):
    ser.close()
    print("Serial closed")


#movement commands
keyboard.on_press_key("p", SerialOff)
keyboard.on_press_key("w", up)
keyboard.on_press_key("s", down)
keyboard.on_press_key("a", left)
keyboard.on_press_key("d", right)
keyboard.on_press_key(" ", fire)

keyboard.on_release_key("w", udpause)
keyboard.on_release_key("s", udpause)
keyboard.on_release_key("a", lrpause)
keyboard.on_release_key("d", lrpause)
keyboard.on_release_key(" ", firepause)

#exit command
keyboard.wait('esc') 



