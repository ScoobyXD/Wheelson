#COM4
import serial
import keyboard
import time

ser = serial.Serial('COM4', 9600, timeout=1)  # timeout prevents hanging
time.sleep(2)  # give the port time to initialize 
print("Serial ready")

aKey = False
dKey = False
wKey = False
sKey = False
space = False


def up(event):
    global wKey
    if(wKey == False):
        ser.write(b'w')  #send 'a' as a byte
        wKey = True
        print('w')

def down(event):
    global sKey
    if(sKey == False):
        ser.write(b's')  #send 'd' as a byte
        sKey = True
        print('s')

def left(event):
    global aKey
    if(aKey == False):
        ser.write(b'a')  #send 'a' as a byte
        aKey = True
        print('a')

def right(event):
    global dKey
    if(dKey == False):
        ser.write(b'd')  #send 'd' as a byte
        dKey = True
        print('d')

def fire(event):
    global space
    if(space == False):
        ser.write(b' ')  #send ' ' as a byte
        space = True
        print(' ')

def upPause(event):
    global wKey
    ser.write(b'y') #send 'y' as a byte
    wKey = False
    print('y')

def downPause(event):
    global sKey
    ser.write(b'y') #send 'y' as a byte
    sKey = False
    print('y')

def leftPause(event):
    global aKey
    ser.write(b'x') #send 'x' as a byte
    aKey = False
    print('x')

def rightPause(event):
    global dKey
    ser.write(b'x') #send 'x' as a byte
    dKey = False
    print('x')

def firePause(event):
    global space
    ser.write(b'z') #send 'z' as a byte
    space = False
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

keyboard.on_release_key("w", upPause)
keyboard.on_release_key("s", downPause)
keyboard.on_release_key("a", leftPause)
keyboard.on_release_key("d", rightPause)
keyboard.on_release_key(" ", firePause)

#exit command
keyboard.wait('esc') 



