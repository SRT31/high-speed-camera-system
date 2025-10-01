import RPi.GPIO as GPIO
import time

# Use BCM numbering (GPIO pins, not physical pin numbers)
GPIO.setmode(GPIO.BCM)

PIN = 17  # BCM pin for capture trigger from Arduino

# Configure pin with pull-down so default state is LOW
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def drop_detected(channel):
    print(f"Drop detected at {time.strftime('%H:%M:%S')}")

# Detect rising edges (LOW to HIGH)
GPIO.add_event_detect(PIN, GPIO.RISING, callback=drop_detected, bouncetime=200)

print("Waiting for signal from Arduino... Press Ctrl+C to exit.")
try:
    while True:
        time.sleep(0.1)  # Keep script alive
except KeyboardInterrupt:
    print("Exiting...")
finally:
    GPIO.cleanup()
