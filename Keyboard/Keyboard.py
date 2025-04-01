#COM4
import serial
import keyboard
import time

try:
    ser = serial.Serial('COM4', 9600, timeout=1)  # timeout prevents hanging

    time.sleep(2)  # give the port time to initialize

    ser.write(b'a')  # send 'a' as a byte

    response = ser.readline().decode().strip()  # optional: read reply from STM32
    print("STM32 says:", response)

    ser.close()

except serial.SerialException as e:
    print("Serial error:", e)



def left(event):
    #send serial command a
    print('a')

def right(event):
    #send serial command d
    print('d')

#movement commands
#keyboard.on_release_key("w", w0)
keyboard.on_press_key("a", left)
keyboard.on_press_key("d", right)


#exit command
keyboard.wait('esc') 



