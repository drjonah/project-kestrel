from gpiozero import LED
from time import sleep

led = LED(14) # GPIO 14 == Pin 8

while True:
    led.toggle()
    sleep(1)
