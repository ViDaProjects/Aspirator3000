import time
import numpy as np
from gpiozero_extended import Motor

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

    def __init__(self, enable_pin, pwm1_pin, pwm2_pin, 
                 encoder1_pin, encoder2_pin, encoder_ppr, wheel_radius):
        self.enable_pin = enable_pin
        self.wheel_radius = wheel_radius

        self.motor = Motor(enable1=enable_pin, pwm1=pwm1_pin, pwm2=pwm2_pin, 
                           encoder1=encoder1_pin, encoder2=encoder2_pin, encoderppr=encoder_ppr)
        
        self.motor.reset_angle()

        self.motor_speed_ratio = 0 
        self.motor_direction = 1 
        self.is_stopped = False #Apagar?    
        self.time_previous = 0 #Apagar?
        self.time_current = 0 #Apagar?
        self.time_start = 0 #time.perf_counter()
        self.angular_vel_current = 0
        self.linear_vel_current = 0
        self.motor_angle_current = 0 #Theta
        self.motor_angle_previous = 0
        self.max_angular_vel = 0

    def stop_motor(self):
        self.is_stopped = True
        self.motor.set_output(0, True)
        self.motor.reset_angle()
        self.motor_speed_ratio = 0
        self.motor_direction = 1

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