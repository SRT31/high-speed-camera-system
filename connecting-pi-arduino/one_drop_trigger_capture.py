#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import subprocess
import threading
import sys
import os

# --- GPIO setup ---
GPIO.setmode(GPIO.BCM)
PIN = 17  # BCM GPIO for capture trigger from Arduino
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# --- one-shot guard ---
captured_once = False
lock = threading.Lock()

def drop_detected(channel):
    global captured_once
    with lock:
        if captured_once:
            return
        captured_once = True  # latch immediately to prevent reentry
        # Stop listening for more edges right away
        try:
            GPIO.remove_event_detect(PIN)
        except RuntimeError:
            pass

    ts = time.strftime('%Y-%m-%d %H:%M:%S')
    print(f"[{ts}] Drop detected! Starting one-shot capture...")

    try:

        # 2) Capture raw frames
        subprocess.run([
            "./raspiraw/raspiraw",
            "-md", "7", "-t", "100",
            "-hd0", "/dev/shm/hd0.32k",
            "-h", "64", "-w", "640",
            "-g","1", "-eus","200","--regs","0171,01","--regs","0170,01",
            "--fps", "900",
            "-sr","1", "-o", "/dev/shm/out.%06d.raw"
        ], check=True)

        # 3) Concatenate header + raw data
        subprocess.run(
            'ls /dev/shm/*.raw | while read i; do cat /dev/shm/hd0.32k "$i" > "$i".all; done',
            shell=True, check=True
        )

        # 4) Convert to TIFFs with dcraw
        subprocess.run(
            'ls /dev/shm/*.all | while read i; do ./dcraw/dcraw -f -o 1 -v -6 -T -K 63 -S 600 -r 1.101 1.034 0.968 0.818 -q 3 -W "$i"; done',
            shell=True, check=True
        )

        print(f"One-shot capture complete")

    except subprocess.CalledProcessError as e:
        print(f"Capture step failed with return code {e.returncode}")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        # Optional: exit automatically after the one-shot
        GPIO.cleanup()
        sys.exit(0)

# Register rising-edge detect (LOW to HIGH) once
GPIO.add_event_detect(PIN, GPIO.RISING, callback=drop_detected, bouncetime=5)

print("Waiting for FIRST drop signal (one-shot). Press Ctrl+C to exit.")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
