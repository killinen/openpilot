#!/usr/bin/env python3

from panda import Panda  # install https://github.com/commaai/panda
import binascii
import argparse
import time
import _thread
import threading

# CAN message ID for 0x4F0
MSG_TRACKED_FRAME_ID = 0x4F0
motor_bus_speed = 500  # i30 baudrate 500kbps
MOTOR_MSG_TS = 0.005  # 10Hz

def heartbeat_thread(p):
    while True:
        try:
            p.send_heartbeat()
            time.sleep(0.1)
        except:
            break

def CAN_tx_thread(p: Panda, bus):
    print("Starting CAN TX thread...")
    while True:
        time.sleep(MOTOR_MSG_TS)

def CAN_rx_thread(p, bus):
    print("Starting CAN RX thread...")
    p.can_clear(bus)  # flush the buffers
    while True:
        time.sleep(MOTOR_MSG_TS / 10)  # read fast enough to clear buffer
        can_recv = p.can_recv()
        for address, _, dat, src in can_recv:
            if src == bus and address == MSG_TRACKED_FRAME_ID:
                print(f"Received message: addr: {address}, bus: {bus}, dat: {binascii.hexlify(dat)}")
                # Store the data for modification later
                global last_received_data
                last_received_data = dat

def getChar():  # Detect key press
    if "_func" not in getChar.__dict__:
        try:
            import msvcrt  # Windows
            getChar._func = msvcrt.getch
        except ImportError:
            import tty, sys, termios  # POSIX
            def _ttyRead():
                fd = sys.stdin.fileno()
                oldSettings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    answer = sys.stdin.read(1)
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, oldSettings)
                return answer
            getChar._func = _ttyRead
    return getChar._func()

def modify_and_send(p: Panda, bus, bit_to_set=None):
    global last_received_data
    if last_received_data is not None:
        modified_data = bytearray(last_received_data)

        if bit_to_set == 'bit0':
            modified_data[0] |= (1 << 0)  # Set bit 1 of byte 0
        elif bit_to_set == 'bit1':
            modified_data[0] |= (1 << 1)  # Set bit 2 of byte 0
        elif bit_to_set == 'bit2':
            modified_data[0] |= (1 << 2)  # Set bit 2 of byte 0
        else:  # When 'm' is pressed, modify byte 3 data
            modified_data[3] |= (1 << 0)  # Set bit 0 of byte 3

        for i in range(100):  # Loop to send the message 5 times
            # Increment the 7-bit rolling number in byte 2 (bits 1-7)
            rolling_number = (modified_data[2] & 0x7F)  # Get bits 1-7
            rolling_number = (rolling_number + 1) & 0x7F  # Increment and wrap around
            modified_data[2] = (modified_data[2] & 0x80) | rolling_number  # Set back to byte 2

            # Send the modified message back
            p.can_send(MSG_TRACKED_FRAME_ID, modified_data, bus)
            print(f"Sent modified message: {binascii.hexlify(modified_data)}")

def cruise_tester(bus):
    global last_received_data
    last_received_data = None

    panda = Panda()
    panda.set_can_speed_kbps(bus, motor_bus_speed)
    panda.set_safety_mode(Panda.SAFETY_ALLOUTPUT)
    panda.set_power_save(False)  # enable all the busses
    _thread.start_new_thread(heartbeat_thread, (panda,))

    tx_t = threading.Thread(target=CAN_tx_thread, args=(panda, bus), daemon=True)
    rx_t = threading.Thread(target=CAN_rx_thread, args=(panda, bus), daemon=True)

    tx_t.start()
    rx_t.start()

    while True:
        c = getChar()
        if c == 'q' or c == '\x03':  # Ctrl+C
            break
        if c == 'm':
            print("Waiting for the next 0x4F0 message to modify...")
            while last_received_data is None:  # Wait for the next message
                time.sleep(0.01)
            modify_and_send(panda, bus)  # Modify all 4 bytes
        elif c == 'w':                  # +/Resume button emulation
            print("Waiting for the next 0x4F0 message to modify...")
            while last_received_data is None:  # Wait for the next message
                time.sleep(0.01)
            modify_and_send(panda, bus, bit_to_set='bit0')  # Set bit 0
        elif c == 's':                  # -/Set button emulation
            print("Waiting for the next 0x4F0 message to modify...")
            while last_received_data is None:  # Wait for the next message
                time.sleep(0.01)
            modify_and_send(panda, bus, bit_to_set='bit1')  # Set bit 1
        elif c == 'c':                  # Cancel button emulation
            print("Waiting for the next 0x4F0 message to modify...")
            while last_received_data is None:  # Wait for the next message
                time.sleep(0.01)
            modify_and_send(panda, bus, bit_to_set='bit2')  # Set bit 2

    print("Disabling output on Panda...")
    panda.set_safety_mode(Panda.SAFETY_SILENT)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CAN message modifier")
    parser.add_argument("--bus", type=int, help="CAN bus id", default=0)
    args = parser.parse_args()
    cruise_tester(args.bus)
