# Importing modules and classes
import time
import numpy as np
from gpiozero_extended import Motor
#from encoder_motor_controller import EncoderMotorController 

# Assigning parameter values
tstop = False  # Sine wave duration (s)
tsample = 0.01  # Sampling period for code execution (s)

speed = 1
direction = 1 
brake_st = False
decrease = False

# Creating motor object using GPIO pins 16, 17, and 18
# (using SN754410 quadruple half-H driver chip)
# Integrated encoder is on GPIO pins 25 and 25
# GPIO pins 24 (Phase A - C1 at encoder) and 25 (Phase B - C2 at encoder)
mymotor = Motor(
    enable1=22, pwm1=17, pwm2=27,
    encoder1=24, encoder2=25, encoderppr=860.67)
mymotor.reset_angle()

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

    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Assigning motor sinusoidal output using the current time step
    #mymotor.set_output(u0 * np.sin((2*np.pi/T) * tcurr))
    thetacurr = mymotor.get_angle()
    wcurr = np.pi/180 * (thetacurr-thetaprev)/(tcurr-tprev) # pi/180 * degree -> degree to rad/s    
    wcurr_m = wcurr * rm
    mymotor.set_output(speed * direction, brake = brake_st)
    print("Output =", mymotor.value)
    print("Angle = {:0.0f} deg".format(thetacurr))
    print("Speed - rad/s =", wcurr)
    print("Speed - m/s =", wcurr_m)
    
    # Updating output arrays
    #t.append(tcurr)
    #theta.append(mymotor.get_angle())
    # Updating previous time value
    tprev = tcurr
    thetaprev = thetacurr
    time.sleep(3)

print('Done.')
# Stopping motor and releasing GPIO pins
mymotor.set_output(0, brake=True)
del mymotor

#branco - vermelho 
#roxo - azul
#pwm +
#sentido horário -ang negativo 
#pwm - 
#sentido anti - ang positivo

#branco - vermelho 
#azul - roxo
#pwm +
#sentido anti horário -ang positivo
#pwm -
#sentido horario - ang negativo

#Certo: vermelho - branco 

#vermelho - branco
#roxo - azul
#pwm +
#sentido anti horário -ang positivo 
#pwm - 
#sentido horario - ang neg
#INVERTI OS ENCODERS 24 E 25

#vermelho - branco
#roxo - azul
#pwm +
#sentido horário -ang neg 
#pwm - 
#sentido anti horario - ang pos