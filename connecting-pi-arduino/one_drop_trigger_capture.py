#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import subprocess
import threading
import sys
import os

# --- GPIO setup ---
GPIO.setmode(GPIO.BCM)
PIN = 17  # <-- change if you wired a different GPIO
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
        # 1) Init camera I2C (twice, per your sequence)
        subprocess.run(["./fork-raspiraw/camera_i2c"], check=True)
        subprocess.run(["./fork-raspiraw/camera_i2c"], check=True)

        # 2) Capture raw frames
        subprocess.run([
            "./fork-raspiraw/raspiraw",
            "-md", "7", "-t", "100",
            "-ts", "/dev/shm/tstamps.csv",
            "-hd0", "/dev/shm/hd0.32k",
            "-h", "64", "-w", "640",
            "--vinc", "1F", "--fps", "975",
            "-sr", "1", "-o", "/dev/shm/out.%06d.raw"
        ], check=True)

        # 3) Concatenate header + raw data
        subprocess.run(
            'ls /dev/shm/*.raw | while read i; do cat /dev/shm/hd0.32k "$i" > "$i".all; done',
            shell=True, check=True
        )

        # 4) Convert to TIFFs with dcraw
        subprocess.run(
            'ls /dev/shm/*.all | while read i; do ~/dcraw/dcraw -f -o 1 -v -6 -T -q 3 -W "$i"; done',
            shell=True, check=True
        )

        # 5) Build concat list for ffmpeg
        with open("/dev/shm/ffmpeg_concats.txt", "w") as f:
            subprocess.run(["python3", "./make_concat.py"], stdout=f, check=True)

        # 6) Encode to MP4 (one-shot name; change as needed)
        out_name = "./video11.mp4"
        subprocess.run([
            "ffmpeg", "-f", "concat", "-safe", "0",
            "-i", "/dev/shm/ffmpeg_concats.txt",
            "-vcodec", "libx265", "-x265-params", "lossless",
            "-crf", "0", "-b:v", "1M", "-pix_fmt", "yuv420p",
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            out_name
        ], check=True)

        print(f"✅ One-shot capture complete: {out_name}")

    except subprocess.CalledProcessError as e:
        print(f"❌ Capture step failed with return code {e.returncode}")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        # Optional: exit automatically after the one-shot
        GPIO.cleanup()
        sys.exit(0)

# Register rising-edge detect (LOW->HIGH) once
GPIO.add_event_detect(PIN, GPIO.RISING, callback=drop_detected, bouncetime=200)

print("Waiting for FIRST drop signal (one-shot). Press Ctrl+C to exit.")
try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
