# Importing modules and classes
import time
import numpy as np
from gpiozero_extended import Motor, PID


mymotor2 = Motor(
    enable1=23, pwm1=5, pwm2=6,
    encoder1=26, encoder2=16, encoderppr=860.67)
mymotor2.reset_angle()

speed = 1
tsample = 0.01

while True:

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
