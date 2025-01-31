import json
import os
import re
import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import requests
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

def upload_to_imgur(image_path, client_id, title="Simple upload", description="This is a simple image upload in Imgur"):
    try:
        if not os.path.exists(image_path):
            raise FileNotFoundError(f"Image file not found: {image_path}")
            
        allowed_extensions = ['.jpg', '.jpeg', '.png', '.gif']
        if not any(image_path.lower().endswith(ext) for ext in allowed_extensions):
            raise ValueError("Unsupported file format. Use JPG, PNG or GIF")
            
        headers = {
            'Authorization': f'Client-ID {client_id}'
        }
        
        with open(image_path, 'rb') as image_file:
            files = {
                'image': image_file,
                'type': 'image',
                'title': title,
                'description': description
            }
            
            response = requests.post(
                'https://api.imgur.com/3/image',
                headers=headers,
                files=files
            )
            
            response.raise_for_status()
            return response.json()
        
    except requests.exceptions.RequestException as e:
        raise Exception(f"Upload failed: {str(e)}")
    except Exception as e:
        raise Exception(f"Unexpected error: {str(e)}")
    

def extract_command(api_response):
    # Получаем строку message из ответа
    message = api_response['message']
    
    # Используем регулярное выражение для поиска Command
    command_pattern = r'Command: ({[^}]+})'
    match = re.search(command_pattern, message)
    
    if match:
        command_str = match.group(1)
        # Преобразуем строку команды в словарь
        try:
            # Заменяем одинарные кавычки на двойные для корректного JSON
            command_str = command_str.replace("'", '"')
            command_dict = json.loads(command_str)
            return command_dict
        except json.JSONDecodeError:
            return None
    return None



def get_numbers(command_dict):
    linear = int(command_dict['linear'])
    angular = int(command_dict['angular'])
    return linear, angular



def gpt_api(image_link):
    try:    
        payload = {
            "chatId": "673ccf943026c3a30d5184b4",
            "assistantCode": "ai2959328",
            "message": "Image analysis request",
            "maxContext": 1,
            "images": [image_link]

        }
        
        headers = {
            'Content-Type': 'application/json',
            'Authorization': 'shds-SzSHtcrUWyyOC3CkIISznUjVOEL'
        }
        
        response = requests.post('https://gptunnel.ru/v1/assistant/chat', headers=headers, json=payload)
        
        if response.status_code == 200:
            print("Success:", response.json())
        else:
            print("Error:", response.status_code, response.text)

        command = extract_command(response.json())
        linear, angular = get_numbers(command)
        print(f'Linear: {linear}, Angular: {angular}')
        return linear, angular

            
    except Exception as e:
        print(f"An error occurred: {str(e)}")

class CameraSubscriber:
    def __init__(self):
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

    def save_image(self, path="camera_image.jpg"):
        """Функция для сохранения изображения"""
        if self.current_image is not None:
            cv2.imwrite(path, self.current_image)
            rospy.loginfo(f"Изображение сохранено в {path}")
        else:
            rospy.logwarn("Нет изображения для сохранения")
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
        2sec = 30°
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

class CameraSubscriber:
    def __init__(self):
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

    def save_image(self, path="camera_image.jpg"):
        """Функция для сохранения изображения"""
        if self.current_image is not None:
            cv2.imwrite(path, self.current_image)
            rospy.loginfo(f"Изображение сохранено в {path}")
        else:
            rospy.logwarn("Нет изображения для сохранения")


def main():
    controller = RobotController()

    images_count = 0
    while not rospy.is_shutdown():
        choice = input("Choose action (q - stop, w - move forward, s - move backward, a - turn left, d - turn right, c - capture image): ")
        try:
            if choice == 'q':
                break
            elif choice == 'w':
                controller.move_forward()
            elif choice == 's':
                controller.move_backward()
            elif choice == 'a':
                controller.turn(direction='left')
            elif choice == 'd':
                controller.turn(direction='right')
            elif choice == 'c':
                camera_sub = CameraSubscriber()
                # Даем немного времени на получение первого изображения
                rospy.sleep(1)
                
                # Получаем изображение
                image = camera_sub.get_image()
                images_count +=1
                if image is not None:
                    print("Изображение получено " + str(images_count) + "й раз")
               
                # Здесь можно выполнить любую обработку изображения
                # Например, сохранить его
                camera_sub.save_image()

            else:
                print("Invalid choice. Please try again.")

            
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass