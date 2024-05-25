# Importing modules and classes
import time
import numpy as np
from gpiozero import RotaryEncoder

# Assigning parameter values
ppr = 300.8  # Pulses Per Revolution of the encoder
tstop = 240  # Loop execution duration (s)
tsample = 0.02  # Sampling period for code execution (s)
tdisp = 0.5  # Sampling period for values display (s)

# Creating encoder object using GPIO pins 24 (Phase A - C1 at encoder) and 25 (Phase B - C2 at encoder)
encoder = RotaryEncoder(24, 25, max_steps=0)

# Initializing previous values and starting main clock
anglecurr = 0
tprev = 0
tcurr = 0
tstart = time.perf_counter()

# Execution loop that displays the current
# angular position of the encoder shaft
print('Running code for', tstop, 'seconds ...')
print('(Turn the encoder.)')
while tcurr <= tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    # Getting angular position of the encoder
    # roughly every `tsample` seconds (deg.)
    anglecurr = 360 / ppr * encoder.steps
    # Printing angular position every `tdisp` seconds
    if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
        print("Angle = {:0.0f} deg".format(anglecurr))
    # Updating previous values
    tprev = tcurr

print('Done.')
# Releasing GPIO pins
encoder.close()


#Girar manualmente para descobrir os steps e PPR
#Provavelmente é por volta de 11 passos
#Para a roda dar 1 volta, o encoder terá que fazer x voltas * passos 
#156
#456
#207
#486
#989
#138
#60 voltas no encoder - 1 volta na roda: 21.600 graus