import time
import numpy as np
from gpiozero_extended import Motor, PID

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
        self.start_timer()
        self.current_time = 0
        self.prev_time = 0
        
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



