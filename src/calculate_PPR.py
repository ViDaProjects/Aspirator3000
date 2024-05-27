#!/usr/bin/python

from gpiozero import Button

ppr = 0

pin_a = Button(24)                      # Rotary encoder pin A connected to GPIO2
pin_b = Button(25)                      # Rotary encoder pin B connected to GPIO3

def pin_a_rising():                    # Pin A event handler
    if pin_b.is_active_: ppr+= 1 # pin B rising while A is active is a clockwise turn

pin_a.when_activated = pin_a_rising      # Register the event handler for pin A

while True:
    print(ppr)