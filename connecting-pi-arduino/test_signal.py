import RPi.GPIO as GPIO
import time

# Use Broadcom pin numbering
GPIO.setmode(GPIO.BCM)

# Choose the GPIO pin number you connected to (example: GPIO17)
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

