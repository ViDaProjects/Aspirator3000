# Importing modules and classes
import time
import numpy as np
from gpiozero import Button
from subprocess import check_call
from gpiozero_extended import Motor

T = 2  # Period of sine wave (s)
tstop = False  # Sine wave duration (s)
tsample = 1.5  # Sampling period for code execution (s)

#Motor controls
speed_btn = Button(6) #marrom
direction_btn = Button(22) #vermelho
reboot_btn = Button(26, hold_time=2) #branco

speed = 0
direction = 1 
brake_st = False
decrease = False

# Creating motor object using GPIO pins 16, 17, and 18
# (using SN754410 quadruple half-H driver chip)
# Integrated encoder is on GPIO pins 25 and 25
# GPIO pins 24 (Phase A - C1 at encoder) and 25 (Phase B - C2 at encoder)
mymotor = Motor(
    enable1=16, pwm1=17, pwm2=27,
    encoder1=24, encoder2=25, encoderppr=860.67)
mymotor.reset_angle()

#def controls
def control_speed():
    global speed, brake_st, decrease
    new_speed = 0
    if speed == 0:
        brake_st = False
        new_speed = 0.1
    else :
        new_speed = speed * 1.25
        if new_speed > 1 :
            new_speed = 1
            decrease = True
    speed = new_speed

def decrease_speed():
    global speed, brake_st, tstop, direction, decrease
    new_speed = 0
    if speed == 1:
        direction = -1
        new_speed = 0.9
    if speed == 0:
        brake_st = True
        tstop = True
        decrease = False
    else :
        new_speed = speed * 0.75
        if new_speed < 0.1 :
            new_speed = 0
            direction = 1
            
    speed = new_speed    

def control_direction():
    global direction
    direction = direction * (-1)

def stop_motor():
    global speed
    global direction
    global mymotor
    global brake_st
    speed = 0
    direction = 1
    mymotor.reset_angle()
    #stop = True
    brake_st = True

def reboot_rasp():
    check_call(['sudo', 'reboot'])

# Pre-allocating output arrays
t = []
theta = []

# Initializing current time step and starting clock
tprev = 0
tcurr = 0
tstart = time.perf_counter()
wcurr = 0
wcurr_m = 0
thetacurr = 0
thetaprev = 0
rm = 0.037 #wheels radius

# Running motor sine wave output
print('Running code for', tstop, 'seconds ...')
while not tstop:
    #Buttons calls
    if reboot_btn.is_active: #branco
        stop_motor()
    if speed_btn.is_active: #marrom
        control_speed()
    if direction_btn.is_active: #vermelho
        control_direction()

    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    thetacurr = mymotor.get_angle()
    wcurr = np.pi/180 * (thetacurr-thetaprev)/(tcurr-tprev) # pi/180 * degree -> degree to rad/s    
    wcurr_m = wcurr * rm # velocity -> m/s

    print("------------------------------------")
    print("PWM = ", mymotor.value)
    print("Angle = ", thetacurr, " deg")
    print("Speed - rad/s =", wcurr)
    #print("Speed - m/s =", wcurr_m)
    print(" ")

    # Updating output arrays
    #t.append(tcurr)
    #theta.append(mymotor.get_angle())
    # Updating previous time value

    tprev = tcurr
    thetaprev = thetacurr
    #time.sleep(1.5)

    if decrease:
        decrease_speed()
    else:
        control_speed()
    
    mymotor.set_output(speed * direction, brake = brake_st)

    

print('Done.')
mymotor.set_output(0, brake=True)
del mymotor

    
"""Atributos:
- velocidade angular/linear do motor
- PDI
- rpm?
- PPR
- raio da roda: 3,7cm - 0.037m
- 1026 graus e 857 steps para uma volta da roda
- 1042 graus e 871 steps
- 75 voltas do encoder - 1029 e 860 steps
- 3090 graus e 2582 em 3 voltas = 1030graus e 860.67 passos/volta  
- diminuiu a v angular  
"""