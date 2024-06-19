#!/usr/bin/env python

import rospy
import time
import numpy as np
from gpiozero_extended import Motor, PID
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf

# Inicialização do nó ROS
rospy.init_node('motor_odometry_node')

# Publicadores de odometria para os dois motores
odom_pub1 = rospy.Publisher('/motor1/odom', Odometry, queue_size=10)
odom_pub2 = rospy.Publisher('/motor2/odom', Odometry, queue_size=10)
odom_broadcaster = tf.TransformBroadcaster()

# Função para criar a mensagem de odometria
def create_odom_msg(x, y, theta, vx, vy, vth, current_time):
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # Posição
    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0
    odom.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, theta))

    # Velocidade
    odom.twist.twist.linear.x = vx
    odom.twist.twist.linear.y = vy
    odom.twist.twist.angular.z = vth

    return odom

# Parâmetros do motor e PID
T = 2  # Period of sine wave (s)
tstop = 5  # Sine wave duration (s)
tsample = 0.05  # Sampling period for code execution (s)

kp2 = 0.02
ki2 = 0.0
kd2 = 0.0
kp1 = 6.85
ki1 = 0.0
kd1 = 0.0
taupid = 0.05

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

pid1 = PID(tsample, kp1, ki1, kd1, umin=0, tau=taupid)
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

x1 = 0.0
y1 = 0.0
theta1 = 0.0

x2 = 0.0
y2 = 0.0
theta2 = 0.0

# Callback function for /cmd_vel
def cmd_vel_callback(msg):
    global wsp
    wsp = msg.linear.x
    # Adjust the motor speeds based on the received velocity commands
    # You can add more complex logic here if necessary

# Subscribe to the /cmd_vel topic
rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)

# Main loop
rate = rospy.Rate(1 / tsample)

while not rospy.is_shutdown() and tcurr < tstop:
    # Pausing for `tsample` to give CPU time to process encoder signal
    time.sleep(tsample)
    # Getting current time (s)
    tcurr = time.perf_counter() - tstart
    current_time = rospy.Time.now()

    thetacurr1 = mymotor.get_angle()
    thetacurr2 = mymotor2.get_angle()

    # Calculating speeds for both motors
    wcurr1 = np.pi / 180 * (thetacurr1 - thetaprev1) / (tcurr - tprev)  # rad/s    
    wcurr2 = np.pi / 180 * (thetacurr2 - thetaprev2) / (tcurr - tprev)  # rad/s

    # Calculate filtered speeds
    wfcurr1 = tau / (tau + tsample) * wfprev1 + tsample / (tau + tsample) * wcurr1
    wfcurr2 = tau / (tau + tsample) * wfprev2 + tsample / (tau + tsample) * wcurr2
    

    # Control signals
    ucurr1 = pid1.control(wsp, wfcurr1)
    ucurr2 = pid2.control(wsp, wfcurr2)

    # Odometry calculation
    dt = tcurr - tprev
    delta_x1 = wfcurr1 * dt * np.cos(theta1)
    delta_y1 = wfcurr1 * dt * np.sin(theta1)
    delta_theta1 = wfcurr1 * dt

    delta_x2 = wfcurr2 * dt * np.cos(theta2)
    delta_y2 = wfcurr2 * dt * np.sin(theta2)
    delta_theta2 = wfcurr2 * dt

    x1 += delta_x1
    y1 += delta_y1
    theta1 += delta_theta1

    x2 += delta_x2
    y2 += delta_y2
    theta2 += delta_theta2

    # Create odometry messages
    odom_msg1 = create_odom_msg(x1, y1, theta1, wfcurr1, 0, wfcurr1, current_time)
    odom_msg2 = create_odom_msg(x2, y2, theta2, wfcurr2, 0, wfcurr2, current_time)

    # Publish odometry messages
    odom_pub1.publish(odom_msg1)
    odom_pub2.publish(odom_msg2)

    # Broadcast transforms
    odom_broadcaster.sendTransform(
        (x1, y1, 0),
        tf.transformations.quaternion_from_euler(0, 0, theta1),
        current_time,
        "base_link1",
        "odom"
    )

    odom_broadcaster.sendTransform(
        (x2, y2, 0),
        tf.transformations.quaternion_from_euler(0, 0, theta2),
        current_time,
        "base_link2",
        "odom"
    )

    # Print statements for debugging
    print("------------------------------------")
    print("Motor 1 - PWM =", mymotor.value)
    print("Motor 1 - Angle =", thetacurr1, "deg")
    print("Motor 1 - Speed - rad/s =", wcurr1)
    print("Motor 1 - Speed - m/s =", wcurr_m1)
    print(" ")
    print("Motor 2 - PWM =", mymotor2.value)
    print("Motor 2 - Angle =", thetacurr2, "deg")
    print("Motor 2 - Speed - rad/s =", wcurr2)
    print("Motor 2 - Speed - m/s =", wcurr_m2)
    print(" ")

    # Update previous values
    tprev = tcurr
    thetaprev1 = thetacurr1
    thetaprev2 = thetacurr2
    wfprev1 = wfcurr1
    wfprev2 = wfcurr2


    # Set motor outputs
    mymotor.set_output(ucurr1)
    mymotor2.set_output(ucurr2)

    rate.sleep()

# Parando os motores
mymotor.set_output(0, brake=True)
mymotor2.set_output(0, brake=True)
del mymotor
del mymotor2
