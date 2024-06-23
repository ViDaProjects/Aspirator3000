# Importing modules and classes
import time
import numpy as np
from gpiozero_extended import Motor, PID

T = 2  # Period of sine wave (s)
tstop = 2  # Sine wave duration (s)
tsample = 0.01  # Sampling period for code execution (s)

kp2 = 7.5
ki2 = 0.3
kd2 = 0.01
kp1 = 7.5
ki1 = 0.3
kd1 = 0.01
taupid = 0.05

speed = 1
direction = 1 
brake_st = False
decrease = False

# Setting general parameters
wsp = 5  # Motor speed set point (rad/s)
tau = 0.1  # Speed low-pass filter response time (s)

mymotor = Motor(
    enable1=22, pwm1=17, pwm2=27,
    encoder1=25, encoder2=24, encoderppr=860.67)
mymotor.reset_angle()


mymotor2 = Motor(
    enable1=23, pwm1=5, pwm2=6,
    encoder1=26, encoder2=16, encoderppr=860.67)
mymotor2.reset_angle()

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

# Pre-allocating output arrays
t1 = []
w1 = []
wf1 = []
u1 = []
t2 = []
w2 = []
wf2 = []
u2 = []

pid = PID(tsample, kp1, ki1, kd1, umin=0, tau=taupid)
pid2 = PID(tsample, kp2, ki2, kd2, umin=0, tau=taupid)

# Initializing current time step and starting clock
tprev = 0
tcurr = 0
tstart = time.perf_counter()
ucurr1 = 0  # x[n] (step input)
wfprev1 = 0  # y[n-1]
wfcurr1 = 0  # y[n]
ucurr2 = 0  # x[n] (step input)
wfprev2 = 0  # y[n-1]
wfcurr2 = 0  # y[n]
wcurr1 = 0
wcurr2 = 0
wcurr_m1 = 0
wcurr_m2 = 0
thetacurr1 = 0
thetacurr2 = 0
thetaprev1 = 0
thetaprev2 = 0
rm = 0.037 #wheels radius

# Running motor sine wave output
print('Running code for', tstop, 'seconds ...')
while tcurr < tstop:

    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    
    thetacurr1 = mymotor.get_angle()
    wcurr1 = np.pi/180 * (thetacurr1-thetaprev1)/(tcurr-tprev) # pi/180 * degree -> degree to rad/s    
    wcurr_m1 = wcurr1 * rm # velocity -> m/s
    wfcurr1 = tau/(tau+tsample)*wfprev1 + tsample/(tau+tsample)*wcurr1
    wfprev1 = wfcurr1
    ucurr1 = pid.control(wsp, wfcurr1)
    t1.append(tcurr)
    w1.append(wcurr1)
    wf1.append(wfcurr1)
    u1.append(ucurr1)


    thetacurr2 = mymotor2.get_angle()
    wcurr2 = np.pi/180 * (thetacurr2-thetaprev2)/(tcurr-tprev) # pi/180 * degree -> degree to rad/s    
    wcurr_m2 = wcurr2 * rm # velocity -> m/s
    wfcurr2 = tau/(tau+tsample)*wfprev2 + tsample/(tau+tsample)*wcurr2
    wfprev2 = wfcurr2
    ucurr2 = pid2.control(wsp, wfcurr2)
    t2.append(tcurr)
    w2.append(wcurr2)
    wf2.append(wfcurr2)
    u2.append(ucurr2)

    print("------------------------------------")
    print("------------------------------------")
    print("PWM = ", mymotor.value)
    print("Angle = ", thetacurr1, " deg")
    print("Speed - rad/s =", wcurr1)
    print("Speed - m/s =", wcurr_m1)
    print(" ")
    print("PWM = ", mymotor2.value)
    print("Angle = ", thetacurr2, " deg")
    print("Speed - rad/s =", wcurr2)
    print("Speed - m/s =", wcurr_m2)
    print(" ")


    tprev = tcurr
    thetaprev1 = thetacurr1
    thetaprev2 = thetacurr2

    mymotor.set_output(ucurr1)
    mymotor2.set_output(ucurr2)
    """if decrease:
        decrease_speed()
    else:
        control_speed()
    
    mymotor.set_output(speed * direction, brake = brake_st)
    mymotor2.set_output(speed * direction, brake = brake_st)
"""
    

print('Done.')
mymotor.set_output(0, brake=True)
mymotor2.set_output(0, brake=True)
del mymotor
del mymotor2

print(t1)
print(" ")
print(w1)
print(" ")
print(wf1)
print(" ")
print(u1)
print("--------------------------------------------")
print("")
print(t2)
print(" ")
print(w2)
print(" ")
print(wf2)
print(" ")
print(u2)    
