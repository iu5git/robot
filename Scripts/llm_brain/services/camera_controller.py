from datetime import datetime
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

class CameraSubscriber:
    def __init__(self, save_path="camera_images"):
        self.save_path = save_path
        self.ensure_save_directory()
        # Инициализируем узел ROS
        # rospy.init_node('camera_subscriber', anonymous=True)
        
        # Создаем объект CvBridge для конвертации между ROS и OpenCV изображениями
        self.bridge = CvBridge()
        
        # Переменная для хранения последнего полученного изображения
        self.current_image = None
        
        # Подписываемся на топик камеры
        self.image_sub = rospy.Subscriber('/mybot/camera/image_raw', 
                                        Image, 
                                        self.image_callback)



    
    def image_callback(self, msg):
        """Callback функция, вызывается при получении нового изображения"""
        try:
            # Конвертируем ROS Image в формат OpenCV
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Ошибка при конвертации изображения: {e}")

    def get_image(self):
        """Функция для получения последнего изображения"""
        if self.current_image is not None:
            return self.current_image.copy()
        else:
            rospy.logwarn("Изображение пока не получено")
            return None

    def save_image(self, path="camera_image.jpg", itteration=0):
        """Функция для сохранения изображения"""
        if self.current_image is not None:
            filename = self.generate_filename(itteration=itteration)
            cv2.imwrite(filename, self.current_image)
            rospy.loginfo(f"Изображение сохранено в {path}")
            return filename
        else:
            rospy.logwarn("Нет изображения для сохранения")

    def generate_filename(self, prefix="camera", itteration=0):
        """
        Генерация имени файла для сохранения изображения.
        
        Args:
            prefix (str): префикс имени файла
            
        Returns:
            str: полное имя файла
        """
        timestamp = datetime.now().strftime("%d_%m_%Y_%H-%M-%S_%f")
        return os.path.join(self.save_path, f"{itteration}_{prefix}_{timestamp}.jpeg")
    
    def ensure_save_directory(self):
        """
        Создание директории для сохранения изображений, если она не существует.
        """
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            rospy.loginfo(f"Создана директория для сохранения: {self.save_path}")
