import RPi.GPIO as GPIO
import time

PIN = 17  # Change to your chosen GPIO pin number (BCM numbering)

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

print("Testing GPIO input... Touch pin to 3.3V to simulate HIGH.")
try:
    while True:
        if GPIO.input(PIN):
            print("HIGH detected!")
        else:
            print("LOW")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("Exiting")
finally:
    GPIO.cleanup()

