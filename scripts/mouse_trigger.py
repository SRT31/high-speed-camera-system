#!/usr/bin/env python3
from evdev import InputDevice, categorize, ecodes
import subprocess
import os

# Replace this with your actual event device
device_path = '/dev/input/event1'
mouse = InputDevice(device_path)

RIGHT_CLICK_CODE = 273  # BTN_RIGHT
MIDDLE_CLICK_CODE = 274  # BTN_MIDDLE

def run_pipeline():
    script = """
rm -rf /dev/shm/tstamps.csv
rm -rf /dev/shm/*.raw
rm -rf /dev/shm/*.tiff
rm -rf /dev/shm/*.all
rm -rf /dev/shm/hd0.32k
sudo rm -rf /dev/shm/ffmpeg_concats.txt

./fork-raspiraw/raspiraw -md 7 -t 100 -ts /dev/shm/tstamps.csv -hd0 /dev/shm/hd0.32k -h 64 -w 640 --vinc 1F --fps 975  -sr 1 -o /dev/shm/out.%06d.raw

ls /dev/shm/*.raw | while read i; do cat /dev/shm/hd0.32k "$i" > "$i".all; done

ls /dev/shm/*.all | while read i; do ~/dcraw/dcraw -f -o 1 -v -6 -T -q 3 -W "$i"; done

python3 ./make_concat.py > /dev/shm/ffmpeg_concats.txt

ffmpeg -f concat -safe 0 -i /dev/shm/ffmpeg_concats.txt -vcodec libx265 -x265-params lossless -crf 0 -b:v 1M -pix_fmt yuv420p -vf "pad=ceil(iw/2)*2:ceil(ih/2)*2" ./video.mp4
"""
    subprocess.Popen(script, shell=True)

print(f"Monitoring mouse events on {device_path}...")

for event in mouse.read_loop():
    if event.type == ecodes.EV_KEY:
        if event.code == RIGHT_CLICK_CODE and event.value == 1:  # key down
            print("Right-click detected. Running pipeline...")
            run_pipeline()
            break
        elif event.code == MIDDLE_CLICK_CODE and event.value == 1:
            print("Middle-click detected. Running pipeline...")
            run_pipeline()
            break
            
print("Done. Exiting.")