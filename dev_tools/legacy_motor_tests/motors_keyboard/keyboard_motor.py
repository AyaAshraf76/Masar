import serial
import sys
import termios
import tty
import time

##################################################
# UART
##################################################

ser = serial.Serial(
    port='/dev/serial0',
    baudrate=115200,
    timeout=1
)

time.sleep(2)

##################################################
# GET KEY
##################################################

def getch():

    fd = sys.stdin.fileno()

    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(
            fd,
            termios.TCSADRAIN,
            old_settings
        )

    return ch

##################################################
# SEND
##################################################

def send(left, right):

    msg = f"{left},{right}\n"

    ser.write(msg.encode())

    print(
        f"\rLeft={left} Right={right}      ",
        end=""
    )

##################################################
# START
##################################################

print("\nControls:")
print("w -> forward")
print("s -> backward")
print("a -> left")
print("d -> right")
print("space -> stop")
print("q -> quit\n")

speed = 120

##################################################
# LOOP
##################################################

while True:

    key = getch()

    # FORWARD
    if key == 'w':

        send( speed,  speed)

    # BACKWARD
    elif key == 's':

        send(-speed, -speed)

    # LEFT
    elif key == 'a':

        send(-speed, speed)

    # RIGHT
    elif key == 'd':

        send(speed, -speed)

    # STOP
    elif key == ' ':

        send(0, 0)

    # QUIT
    elif key == 'q':

        send(0, 0)

        break

##################################################
# CLOSE
##################################################

ser.close()

print("\nStopped")
