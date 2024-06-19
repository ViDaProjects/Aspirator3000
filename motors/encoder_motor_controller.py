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

    def __init__(self, enable_pin_l, enable_pin_r, pwm1_pin_l, pwm2_pin_l, pwm1_pin_r, pwm2_pin_r, 
                 encoder1_pin_l, encoder2_pin_l, encoder1_pin_r, encoder2_pin_r, encoder_ppr_l, encoder_ppr_r, wheel_radius):
        self.wheel_radius = wheel_radius

        self.motor_left = Motor(enable1=enable_pin_l, pwm1=pwm1_pin_l, pwm2=pwm2_pin_l, 
                           encoder1=encoder1_pin_l, encoder2=encoder2_pin_l, encoderppr=encoder_ppr_l)
        
        self.motor_left.reset_angle()

        self.motor_right = Motor(enable1=enable_pin_r, pwm1=pwm1_pin_r, pwm2=pwm2_pin_r, 
                                 encoder1=encoder1_pin_r, encoder2=encoder2_pin_r, encoderppr=encoder_ppr_r)
        self.motor_right.reset_angle()

        pid = PID(self.tsample, self.kpl, self.kil, self.kdl, umin=0, tau=self.taupid)
        pid2 = PID(self.tsample, self.kpr, self.kir, self.kdr, umin=0, tau=self.taupid)

        self.motor_output_left = 0
        self.motor_output_right = 0

        self.is_stopped = False #Apagar?    
        self.time_previous = 0 #Apagar?
        self.time_current = 0 #Apagar?
        self.time_start = time.perf_counter()
        self.angular_vel_current = 0
        self.linear_vel_current = 0
        self.motor_angle_current = 0 #Theta
        self.motor_angle_previous = 0
        self.max_angular_vel = 0

    def stop_motor(self):
        self.is_stopped = True
        self.motor_left.set_output(0, True)
        self.motor_right.set_output(0, True)
        self.motor_left.reset_angle()
        self.motor_right.reset_angle()
        self.motor_output_left = 0
        self.motor_output_right = 0

    #def calculate_angular_vel(self):

    #def get_angular_vel(self):

    def calculate_linear_vel(self):
        self.linear_vel_current = self.angular_vel_current * self.wheel_radius

    def get_linear_vel(self):
        return self.linear_vel_current

    def set_pwm_output(self, angular_vel: int, direction: int):
        self.motor_speed_ratio = angular_vel/self.max_angular_vel
        self.motor_direction = direction
        self.motor.set_output(self.motor_speed_ratio * self.motor_direction)

    #def define_max_vel(self):

    
    def get_current_time(self):
        return self.time_current    

    def set_current_time(self):
        self.time_current = time.perf_counter() - self.time_start
    
    def start_timer(self):
        self.start_timer = time.perf_counter()

    def reset_timer(self):
        self.time_start = 0
        self.time_current = 0
        self.time_previous = 0

    def set_clockwise_dir(self):
        self.motor_direction = 1

    def set_anticlockwise_dir(self):
        self.motor_direction = -1

    def delete_motor(self): 
        del motor


    #PID calculus    

    #Ver o CMD vel -> como que vou receber e lidar com esses dados?