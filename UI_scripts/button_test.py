#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time

PIN = 27  # BCM numbering

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Enable internal pull-up

print("Press the button connected to GPIO17 (CTRL+C to exit)")

last_state = GPIO.input(PIN)

try:
    while True:
        state = GPIO.input(PIN)
        if state != last_state:
            if state == GPIO.LOW:
                print("Button pressed!")
            else:
                print("Button released")
            last_state = state
        time.sleep(0.05)  # debounce delay
except KeyboardInterrupt:
    GPIO.cleanup()
    print("Bye")
