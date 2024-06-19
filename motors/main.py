import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
from encoder_motor_controller import EncoderMotorController 
import tf

rate = 0

#função que inicializa as coisas da odometria
def init_odometry(tsample):
    global rate
    # Inicialização do nó ROS
    rospy.init_node('motor_odometry_node')
    #odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    #odom_broadcaster = tf.TransformBroadcaster()
    
    # Subscribe to the /cmd_vel topic
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    rate = rospy.Rate(1 / tsample)

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

# Callback function for /cmd_vel
def cmd_vel_callback(msg):
    global speed_goal_global
    speed_goal_global = msg.angular.z
    # Adjust the motor speeds based on the received velocity commands
    # You can add more complex logic here if necessary


speed_goal_global = 0

encoder_motor_controller = EncoderMotorController(enable_pin_l=22, enable_pin_r=23, pwm1_pin_l=17, pwm2_pin_l=27, pwm1_pin_r=5, pwm2_pin_r=6, encoder1_pin_l=25, encoder2_pin_l=24, encoder1_pin_r=26, encoder2_pin_r=16, encoder_ppr_l=860.67, encoder_ppr_r=860.67, wheel_radius=0.037)

init_odometry(encoder_motor_controller.get_sampling_period)

while not rospy.is_shutdown():
    # Pausing for `tsample` to give CPU time to process encoder signal
    #time.sleep(tsample)
    # Getting current time (s)
    
    #set speed goal class by wsp global
    current_time = encoder_motor_controller.get_current_time()

    encoder_motor_controller.set_speed_goal(speed_goal_global)

    encoder_motor_controller.calculate_speeds()

    # Control signals
    encoder_motor_controller.calculate_pwm_output()

    encoder_motor_controller.update_previous_values()

    rate.sleep()

    # Odometry calculation
"""    dt = tcurr - tprev
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
"""


# Parando os motores
encoder_motor_controller.stop_motor()
encoder_motor_controller.delete_motor()
