import rospy
import time
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Quaternion
import tf

from gpiozero import DigitalOutputDevice, PWMOutputDevice, RotaryEncoder


class Motor:
    """
    The class to represent a DC motor controlled with an H-bridge driver.
    An encoder can be used to measure the angular position of the output shaft.

    Two types of drivers are allowed:

        * Single enable with dual PWM for forward and backward rotation control
        * Single PWM with dual enable for forward and backward rotation control

    Set up a motor with single enable:
    (SN754410 quadruple half-H driver chip)

        >>> from gpiozero_extended import Motor
        >>> mymotor = Motor(enable1=16, pwm1=17, pwm2=18)

    Set up a motor with single pwm:
    (L298 dual H-bridge motor speed controller board)

        >>> from gpiozero_extended import Motor
        >>> mymotor = Motor(enable1=16, enable2=17, pwm1=18)

    Set up a motor with single enable and an encoder with 450 PPR:
    (SN754410 quadruple half-H driver chip)

        >>> from gpiozero_extended import Motor
        >>> mymotor = Motor(
            enable1=16, pwm1=17, pwm2=18,
            encoder1=24, encoder2=25, encoderppr=450)

    :param enable1: The GPIO pin that is connected to the enable 1 of the driver.
    :type enable1: int or str
    :param enable2: The GPIO pin that is connected to the enable 2 of the driver.
        This value is ignored for a single enable driver.
    :type enable2: int or str
    :param pwn1: The GPIO pin that is connected to the PWM 1 of the driver.
    :type pwm1: int or str
    :param pwm2: The GPIO pin that is connected to the PWM 2 of the driver.
        This value is ignored for a single PWM driver.
    :type pwm2: int or str
    :param encoder1: The GPIO pin that is connected to the encoder phase A.
    :type encoder1: int or str
    :param encoder2: The GPIO pin that is connected to the encoder phase B.
    :type encoder2: int or str
    :param encoderppr: The number of Pulses Per Revolution (PPR) of the encoder.
        Default value is ``300``.
    :type encoderppr: int

    .. note::
        Always use `del` to delete the motor object after it's used to
        release the GPIO pins.

    """

    def __init__(
        self, enable1=None, enable2=None, pwm1=None, pwm2=None,
        encoder1=None, encoder2=None, encoderppr=300):
        """
        Class constructor.

        """
        # Identifying motor driver type and assigning appropriate GPIO pins
        if pwm1 and pwm2:
            # Driver with 1 enable and 2 PWM inputs
            # Example: SN754410 quadruple half-H driver chip
            if not enable1:
                raise Exception('"enable1" pin is undefined.')
            self._dualpwm = True
            self._enable1 = DigitalOutputDevice(enable1)
            self._pwm1 = PWMOutputDevice(pwm1)
            self._pwm2 = PWMOutputDevice(pwm2)
        elif enable1 and enable2:
            # Driver with 2 enables and 1 PWM input
            # Example: L298 dual H-bridge motor speed controller board
            if not pwm1:
                raise Exception('"pwm1" pin is undefined.')
            self._dualpwm = False
            self._enable1 = DigitalOutputDevice(enable1)
            self._enable2 = DigitalOutputDevice(enable2)
            self._pwm1 = PWMOutputDevice(pwm1)
        else:
            raise Exception('Pin configuration is incorrect.')
        # Checking for encoder
        if encoder1 and encoder2:
            self._encoder = RotaryEncoder(encoder1, encoder2, max_steps=0)
            self._ppr = encoderppr
        else:
            self._encoder = None
        # Initializing attributes
        self._value = 0  # Motor output value
        self._angle0 = 0  # Initial angular position

    def __del__(self):
        """
        Class destructor.
        
        """
        # Releasing GPIO pins
        if self._dualpwm:
            self._enable1.close()
            self._pwm1.close()
            self._pwm2.close()
        else:
            self._enable1.close()
            self._enable2.close()
            self._pwm1.close()
        if self._encoder:
            time.sleep(1)  # Added this to avoid segmentation fault :(
            self._encoder.close()

    @property
    def value(self):
        """
        Contains the motor output level (`read only`).
        Values can be between ``-1`` (full speed backward) and ``1`` (full
        speed forward), with ``0`` being stopped.

        """
        return self._value

    @value.setter
    def value(self, _):
        print('"value" is a read only attribute.')

    def get_angle(self):
        """
        Get the value of the encoder output angle.

        >>> mymotor.get_angle()

        """
        if self._encoder:
            angle = 360 / self._ppr * self._encoder.steps - self._angle0
        else:
            angle = None
        return angle

    def reset_angle(self):
        """
        Reset the encoder output angle.

        >>> mymotor.reset_angle()

        """
        if self._encoder:
            self._angle0 = 360 / self._ppr * self._encoder.steps

    def set_output(self, output, brake=False):
        """
        Set motor output.

        :param output: The PWM duty cycle value between ``-1`` and ``1``.
            A value of ``0`` stops the motor.
        :type output: float

        :param brake: The motor brake option used when duty cycle is zero.
            Brake is applied when ``True``. Motor is floating when ``False``.
        :type brake: bool

        Set output to ``0.5``:

            >>> mymotor.set_output(0.5)

        Set output to ``0.25`` (reverse rotation):

            >>> mymotor.set_output(-0.25)

        Stop motor and apply brake:

            >>> mymotor.set_output(0, brake=True)

        """
        # Limiting output
        if output > 1:
            output = 1
        elif output < -1:
            output = -1
        # Forward rotation
        if output > 0:
            if self._dualpwm:
                self._enable1.on()
                self._pwm1.value = output
                self._pwm2.value = 0
            else:
                self._enable1.on()
                self._enable2.off()
                self._pwm1.value = output
        # Backward rotation
        elif output < 0:
            if self._dualpwm:
                self._enable1.on()
                self._pwm1.value = 0
                self._pwm2.value = -output
            else:
                self._enable1.off()
                self._enable2.on()
                self._pwm1.value = -output
        # Stop motor
        elif output == 0:
            if brake:
                if self._dualpwm:
                    self._enable1.off()
                    self._pwm1.value = 0
                    self._pwm2.value = 0
                else:
                    self._enable1.off()
                    self._enable2.off()
                    self._pwm1.value = 0
            else:
                if self._dualpwm:
                    self._enable1.on()
                    self._pwm1.value = 0
                    self._pwm2.value = 0
                else:
                    self._enable1.on()
                    self._enable2.on()
                    self._pwm1.value = 0
        # Updating output value property
        self._value = output


class DAC:
    """
    The class to represent a DAC port.

    For the Digital-to-Analog port to work properly, the GPIO pin output must
    be the input of an analog low-pass filter. An easy filter implementation
    can be achieved with two cascaded passive RC filters, with R = 1 kOhm and
    C = 10 uF. 
    
    For more information on how to make the filter go to:
    https://thingsdaq.org/2022/02/08/dac-with-raspberry-pi/


    Set up a DAC port on GPIO pin 18:

        >>> from gpiozero_extended import DAC
        >>> mydac = DAC(dacpin=18)

    :param dacpin: The GPIO pin that is used for the DAC port.
    :type dacpin: int or str

    .. note::
        Always use `del` to delete the DAC object after it's used to
        release the GPIO pin.

    """
    def __init__(self, dacpin=12):
        """
        Class constructor.

        """
        # Checking for valid hardware PWM pins
        if dacpin not in [12, 13, 18, 19]:
            raise Exception('Valid GPIO pin is: 12, 13, 18, or 19')
        # Assigning attributes
        self._vref = 3.3  # Reference voltage output
        self._slope = 1  # Output transfer function slope
        self._offset = 0  # Output transfer function intercept
        # Creating PWM pin object
        self._dac = PWMOutputDevice(dacpin, frequency=700)

    def __del__(self):
        """
        Class destructor.
        
        """
        # Releasing GPIO pin
        self._dac.close()

    def reset_calibration(self):
        """
        Reset the DAC calibration:

            * ``slope``=1
            * ``offset``=0

        >>> mydac.reset_calibration()

        """
        self._slope = 1
        self._offset = 0

    def set_calibration(self, slope, offset):
        """
        Set the DAC calibration ``slope`` and ``offset`` values.
        
        The ``slope`` and ``offset`` (intercept) values can be determined
        by using the method `set_output' and an external DAQ device capable
        of measuring the low-pass filter voltage output.

        A two-point calibration (output1, voltage1), (output2, voltage2) can be
        used to solve the system of equations below for ``slope`` and
        ``offset``:

            output1 = ``slope`` * voltage1 + ``offset``
            output2 = ``slope`` * voltage2 + ``offset``

        Or:

            ``slope`` = (output2-output1)/(voltage2-voltage1)
            ``offset`` = output1 - (output2-output1)/(voltage2-voltage1)*voltage1


        Set the ``slope`` to 1.0451 and the ``offset`` to -0.0673:
        >>> mydac.set_calibration(1.0451, -0.0673)

        """
        self._slope = slope
        self._offset = offset

    def get_calibration(self):
        """
        Print the current calibration ``slope`` and ``offset`` values.
        
        >>> mydac.get_calibration()

        """
        print('Slope = {:0.4f} , Offset = {:0.4f}'.format(
            self._slope, self._offset))

    def set_output(self, value):
        """
        Set the DAC output voltage.
        The output value is limited between ``0`` anf ``vref``

        Set an out voltage of 2.5 V:
        >>> mydac.set_output(2.5)

        """
        # Limiting output
        output = (self._slope*value + self._offset)/self._vref
        if output > 1:
            output = 1
        if output < 0:
            output = 0
        # Applying output to GPIO pin
        self._dac.value = output


class PID:
    """
    The class to represent a discrete PID controller.

    For more information about this class go to:
    https://thingsdaq.org/2022/04/07/digital-pid-controller/


    Create a PID controller:

        >>> Ts = 0.01
        >>> kp = 0.15
        >>> ki = 0.35
        >>> kd = 0.01
        >>> mypid = PID(Ts, kp, ki, kd)

    :param Ts: The sampling period of the execution loop.
    :type Ts: float

    :param kp: The PID proportional gain.
    :type kp: float

    :param ki: The PID integral gain.
    :type ki: float

    :param kd: The PID derivative gain.
    :type kd: float

    :param umax: The upper bound of the controller output saturation.
        Defalt value is ``1``.
    :type umax: float

    :param umin: The lower bound of the controller output saturation.
        Defalt value is ``-1``.
    :type umin: float

    :param tau: The derivative term low-pass filter response time (s).
        Defalt value is ``0``.
    :type tau: float

    """
    def __init__(self, Ts, kp, ki, kd, umax=1, umin=-1, tau=0):
        """
        Class constructor.

        """
        self._Ts = Ts  # Sampling period (s)
        self._kp = kp  # Proportional gain
        self._ki = ki  # Integral gain
        self._kd = kd  # Derivative gain
        self._umax = umax  # Upper output saturation limit
        self._umin = umin  # Lower output saturation limit
        self._tau = tau  # Derivative term filter time constant (s)
        #
        self._eprev = [0, 0]  # Previous errors e[n-1], e[n-2]
        self._uprev = 0  # Previous controller output u[n-1]
        self._udfiltprev = 0  # Previous filtered value

    def control(self, xsp, x, uff=0):
        """
        Calculate PID controller output.

        :param xsp: The set point value at the time step.
        :type xsp: float

        :param x: The actual value at the time step.
        :type x: float

        :param uff: The feed-forward value at the time step.
            Default value is ``0``.
        :type uff: float

        """
        # Calculating error
        e = xsp - x
        # Calculating proportional term
        up = self._kp * (e - self._eprev[0])
        # Calculating integral term (with anti-windup)
        ui = self._ki*self._Ts * e
        if (self._uprev+uff >= self._umax) or (self._uprev+uff <= self._umin):
            ui = 0
        # Calculating derivative term
        ud = self._kd/self._Ts * (e - 2*self._eprev[0] + self._eprev[1])
        # Filtering derivative term
        udfilt = (
            self._tau/(self._tau+self._Ts)*self._udfiltprev +
            self._Ts/(self._tau+self._Ts)*ud
        )
        # Calculating PID controller output
        u = self._uprev + up + ui + udfilt + uff
        # Updating previous time step errors
        self._eprev[1] = self._eprev[0]
        self._eprev[0] = e
        # Updating previous time step output value
        self._uprev = u - uff
        # Updating previous time step derivative term filtered value
        self._udfiltprev = udfilt
        # Limiting output (just to be safe)
        if u < self._umin:
            u = self._umin
        elif u > self._umax:
            u = self._umax
        # Returning controller output at current time step
        return u

class EncoderMotorController:
    #tsample = 0.01  # Sampling period for code execution (s) is needed?
    
    """
    wheel_radius => m
    pwm1_pin => #IN1 on H-bridge 
    pwm2_pin => #IN2 on H-bridge
    encoder1_pin => #Phase A - C1 at encoder
    encoder2_pin => #Phase B - C2 at encoder   
    motor_speed_ratio => #value between 0 and 1 that represents percentage
    motor_direction => #1: clockwise, -1: anti-clockwise
    angular_vel_current => rad/s
    linear_vel_current => m/s  
    max_angular_vel => Max motor velocity   
    """

    tsample = 0.01
    kpl = 7.5
    kil = 0.3
    kdl = 0.01
    kpr = 7.5
    kir = 0.3
    kdr = 0.01
    taupid = 0.05
    tau = 0.1 #filter response time (s)
    sampling_period = 0.05

    def __init__(self, enable_pin_l, enable_pin_r, pwm1_pin_l, pwm2_pin_l, pwm1_pin_r, pwm2_pin_r, 
                 encoder1_pin_l, encoder2_pin_l, encoder1_pin_r, encoder2_pin_r, encoder_ppr_l, encoder_ppr_r, wheel_radius):
        
        #Motor data
        self.wheel_radius = wheel_radius

        self.motor_left = Motor(enable1=enable_pin_l, pwm1=pwm1_pin_l, pwm2=pwm2_pin_l, 
                           encoder1=encoder1_pin_l, encoder2=encoder2_pin_l, encoderppr=encoder_ppr_l)
        
        self.motor_left.reset_angle()

        self.motor_right = Motor(enable1=enable_pin_r, pwm1=pwm1_pin_r, pwm2=pwm2_pin_r, 
                                 encoder1=encoder1_pin_r, encoder2=encoder2_pin_r, encoderppr=encoder_ppr_r)
        self.motor_right.reset_angle()

        self.pid1 = PID(self.tsample, self.kpl, self.kil, self.kdl, umin=0, tau=self.taupid)
        self.pid2 = PID(self.tsample, self.kpr, self.kir, self.kdr, umin=0, tau=self.taupid)

        self.motor_output_left = 0
        self.motor_output_right = 0

        self.theta_curr_left = 0
        self.theta_curr_right = 0
        self.theta_prev_left = 0 
        self.theta_prev_right = 0

        #Time
        self.time_start = time.perf_counter()
        self.current_time = time.perf_counter()
        self.prev_time = 0.01
        
        #Velocities
        self.speed_goal = 0
        self.filt_speed_ang_curr_left = 0
        self.filt_speed_ang_curr_right = 0
        self.filt_speed_ang_prev_left = 0
        self.filt_speed_ang_prev_right = 0
        self.speed_ang_curr_left = 0
        self.speed_ang_curr_right = 0


        self.is_stopped = False #Apagar?    

    def stop_motor(self):
        self.is_stopped = True
        self.motor_left.set_output(0, True)
        self.motor_right.set_output(0, True)
        self.motor_left.reset_angle()
        self.motor_right.reset_angle()
        self.motor_output_left = 0
        self.motor_output_right = 0

    def calculate_linear_vel(self):
        self.linear_vel_current = self.angular_vel_current * self.wheel_radius

    def get_linear_vel(self):
        return self.linear_vel_current

    def set_speed_goal(self, speed):
        self.speed_goal = speed

    def calculate_pwm_output(self):
        self.motor_output_left = self.pid1.control(self.speed_goal, self.filt_speed_ang_curr_left)
        self.motor_output_right = self.pid2.control(self.speed_goal, self.filt_speed_ang_curr_right)

    #calculate speeds and filtered speeds -> chama isso dentro do pwm output?
    def calculate_speeds(self):
        self.theta_curr_left = self.motor_left.get_angle()
        self.theta_curr_right = self.motor_right.get_angle()

        if self.current_time - self.prev_time == 0: 
            self.speed_ang_curr_left = 0
            self.speed_ang_curr_right = 0

        else: 
            self.speed_ang_curr_left = np.pi / 180 * (self.theta_curr_left - self.theta_prev_left) / (self.current_time - self.prev_time)  # rad/s    
            self.speed_ang_curr_right = np.pi / 180 * (self.theta_curr_right - self.theta_prev_right) / (self.current_time - self.prev_time)

        # Calculate filtered speeds
        self.filt_speed_ang_curr_left = self.tau / (self.tau + self.sampling_period) * self.filt_speed_ang_curr_left + self.sampling_period / (self.tau + self.sampling_period) * self.speed_ang_curr_left
        self.filt_speed_ang_curr_right = self.tau / (self.tau + self.sampling_period) * self.filt_speed_ang_curr_right + self.sampling_period / (self.tau + self.sampling_period) * self.speed_ang_curr_right
        
    def get_sampling_period(self):
        return self.sampling_period()

    #Calculate odometry message

    #Create odometry message

    #Publish odometry message

    #Broadcast transforms (o que é???)

    def update_previous_values(self):
        self.filt_speed_ang_prev_left = self.filt_speed_ang_curr_left
        self.filt_speed_ang_prev_right = self.filt_speed_ang_curr_right
        self.prev_time = self.current_time
        self.theta_prev_left = self.theta_curr_left
        self.theta_prev_right = self.theta_curr_right

    def get_current_time(self):
        self.time_current = time.perf_counter() - self.time_start
        return self.time_current    


    def start_timer(self):
        self.start_timer = time.perf_counter()

    def reset_timer(self):
        self.time_start = 0
        self.time_current = 0
        self.time_previous = 0

    def delete_motor(self): 
        del self.motor_left
        del self.motor_right






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
    rate = rospy.Rate(20)

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
