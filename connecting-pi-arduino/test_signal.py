import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

PIN = 17

# Set pin as input with pull-down resistor
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

print("Waiting for signal from Arduino...")

try:
    while True:
        if GPIO.input(PIN) == GPIO.HIGH:
            print("Drop detected!")
        time.sleep(0.01)  # small delay to avoid CPU overuse

except KeyboardInterrupt:
    print("Exiting...")

finally:
    GPIO.cleanup()

