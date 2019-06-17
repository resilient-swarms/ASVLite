from evdev import InputDevice, categorize, ecodes
gamepad = InputDevice('/dev/input/event0') #assigning the joystick to the correct input
#Listing the values of the event codes for each button and analogue stick
X = 304
Square = 307
Circle = 305
Triangle = 308
R1 = 311
R2 = 313
L1 = 310
L2 = 312
Start = 315
Select = 314
Analog = 316
R3 = 318
L3 = 317
LX = 00
RX = 2
LY = 1
RY = 5
lx = 16
ly = 17
#For loops printing the button that has been pressed
for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        if event.value == 1:
            if event.code == X:
                print("X")
            elif event.code == Square:
                print("Square")
            elif event.code == Circle:
                print("Circle")
            elif event.code == Triangle:
                print("Triangle")
            elif event.code == R1:
                print("R1")
            elif event.code == R2:
                print("R2")
            elif event.code == L1:
                print("L1")
            elif event.code == L2:
                print("L2")
            elif event.code == Start:
                print("Start")
            elif event.code == Select:
                print("Select")
            elif event.code == Analog:
                print("Analog")
            elif event.code == L3:
                print("L3")
            elif event.code == R3:
                print("R3")
    elif event.type == ecodes.EV_ABS: #For loop for analogue sticks, currently the Left X axis is been played with for motor output
            if event.code == LX:
                if event.value > 128:
                    print("LX" + str(event.value))
                elif event.value <128:
                    print("LX" + str(event.value))
            if event.code == LY:
                if event.value < 127:
                    print("Left Stick Up")
                elif event.value >127:
                    print("Left Stick Down")
            if event.code == RX:
                if event.value > 128:
                    print("Right Stick Right")
                elif event.value <128:
                    print("Right Stick Left")
            if event.code == RY:
                if event.value < 127:
                    print("Right Stick Up")
                elif event.value >127:
                    print("Right Stick Down")
            if event.code == lx:
                if event.value == 1:
                    print("Arrow Right")
                elif event.value == -1:
                    print("Arrow Left")
            if event.code == ly:
                if event.value == 1:
                    print("Arrow Down")
                elif event.value == -1:
                    print("Arrow Up")
