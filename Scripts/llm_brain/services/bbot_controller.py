import json
import os
import random
import re
import string
import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
import os
import requests
from typing import Union, Dict, Any


class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_speed = 0.0
        
    def smooth_acceleration(self, target_speed, acceleration_time=2.0):
        """
        Плавный разгон до целевой скорости
        :param target_speed: целевая скорость
        :param acceleration_time: время разгона в секундах
        """
        start_time = time.time()
        start_speed = self.current_speed
        
        while time.time() - start_time < acceleration_time:
            # Вычисляем текущее время с начала разгона
            current_time = time.time() - start_time
            
            # Используем синусоидальную функцию для плавного изменения скорости
            progress = current_time / acceleration_time
            speed = start_speed + (target_speed - start_speed) * (1 - np.cos(progress * np.pi / 2))
            
            twist = Twist()
            twist.linear.x = speed
            self.cmd_vel_pub.publish(twist)
            self.current_speed = speed
            
            self.rate.sleep()
    
    def smooth_deceleration(self, deceleration_time=2.0):
        """
        Плавное торможение до полной остановки
        :param deceleration_time: время торможения в секундах
        """
        start_time = time.time()
        start_speed = self.current_speed
        
        while time.time() - start_time < deceleration_time:
            # Вычисляем текущее время с начала торможения
            current_time = time.time() - start_time
            
            # Используем синусоидальную функцию для плавного торможения
            progress = current_time / deceleration_time
            speed = start_speed * np.cos(progress * np.pi / 2)
            
            twist = Twist()
            twist.linear.x = speed
            self.cmd_vel_pub.publish(twist)
            self.current_speed = speed
            
            self.rate.sleep()
        
        # Убеждаемся, что робот полностью остановился
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        self.current_speed = 0.0

    def move_forward(self, speed=-0.2, duration=2.0):
        """
        Движение вперед с плавным разгоном и торможением
        2sec=60cm
        """
        self.smooth_acceleration(speed)
        time.sleep(duration)
        self.smooth_deceleration()

    def move_backward(self, speed=0.2, duration=2.0):
        """
        Движение назад с плавным разгоном и торможением
        """
        self.smooth_acceleration(speed)
        time.sleep(duration)
        self.smooth_deceleration()

    def turn(self, angular_speed=0.5, duration=2.0, direction='left'):
        """
        Поворот с плавным изменением угловой скорости
        """
        twist = Twist()
        if direction == 'left':
            twist.angular.z = angular_speed
        else:
            twist.angular.z = -angular_speed
            
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
            
        # Остановка поворота
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
