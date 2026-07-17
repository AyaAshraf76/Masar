import serial
import time

ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=115200,
    timeout=0.1
)

time.sleep(2)

print("===================================")
print("UART CONTROL STARTED")
print("===================================")

while True:

    print("\nChoose Mode:")
    print("1 -> 4 Wheel Speed Control")
    print("2 -> Steering + Speed Control")
    print("q -> Quit")

    mode = input("Enter mode: ")

    if mode.lower() == 'q':
        break

    ####################################################
    # MODE 1
    # 1,m1,m2,m3,m4
    ####################################################

    if mode == '1':

        print("\nEnter:")
        print("m1 m2 m3 m4")

        try:

            user_input = input("\nTargets: ")

            m1, m2, m3, m4 = user_input.split()

            msg = f"1,{m1},{m2},{m3},{m4}\n"

            ser.write(msg.encode())

            print("Sent:", msg.strip())

        except:
            print("Invalid input!")

    ####################################################
    # MODE 2
    # 2,steer,speed
    ####################################################

    elif mode == '2':

        print("\nEnter:")
        print("steer speed")

        print("\nSteer range:")
        print("-1.0 -> Full Left")
        print(" 0.0 -> Center")
        print("+1.0 -> Full Right")

        try:

            user_input = input("\nCommand: ")

            steer, speed = user_input.split()

            msg = f"2,{steer},{speed}\n"

            ser.write(msg.encode())

            print("Sent:", msg.strip())

        except:
            print("Invalid input!")

    else:
        print("Invalid mode!")
        continue

    ####################################################
    # RECEIVE FEEDBACK
    ####################################################

    time.sleep(0.05)

    while ser.in_waiting:

        line = ser.readline().decode(
            errors='ignore'
        ).replace('\x00', '').strip()

        if line:

            try:

                values = line.split(',')

                if len(values) == 10:

                    rpm1 = float(values[0])
                    rpm2 = float(values[1])
                    rpm3 = float(values[2])
                    rpm4 = float(values[3])

                    pos1 = int(values[4])
                    pos2 = int(values[5])
                    pos3 = int(values[6])
                    pos4 = int(values[7])

                    modeFeedback = int(values[8])

                    vehicleSpeed = float(values[9])

                    print("\n========== FEEDBACK ==========")

                    print(f"RPM1: {rpm1:.2f}")
                    print(f"RPM2: {rpm2:.2f}")
                    print(f"RPM3: {rpm3:.2f}")
                    print(f"RPM4: {rpm4:.2f}")

                    print(f"POS1: {pos1}")
                    print(f"POS2: {pos2}")
                    print(f"POS3: {pos3}")
                    print(f"POS4: {pos4}")

                    print(f"MODE: {modeFeedback}")

                    print(f"Vehicle Speed RPM: {vehicleSpeed:.2f}")

                    print("================================")

            except:
                pass

ser.close()
