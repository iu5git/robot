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
API_CONFIG = {
    'BASE_URL': 'http://193.XX.XX.XX:8011',
    'UPLOAD_ENDPOINT': '/api/v1/upload',
    'HEADERS': {
        'accept': 'application/json'
    }
}

import requests
from typing import List, Dict, Union, Optional


def get_all_commands(skip=0, limit=100):
    """
    Получает список всех команд с пагинацией
    
    Args:
        skip (int): Количество пропускаемых записей
        limit (int): Максимальное количество возвращаемых записей
    
    Returns:
        list: Список команд или None в случае ошибки
    """
    try:
        url = f"http://193.XX.XX.XX:8011/commands/?skip={skip}&limit={limit}"
        response = requests.get(url)
        
        if response.status_code == 200:
            return response.json()
        else:
            print(f"Ошибка при получении команд: {response.status_code}")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"Ошибка при выполнении запроса: {e}")
        return None

def add_command_to_history(command):
    """
    Добавляет новую команду в историю
    
    Args:
        command (dict): Словарь с данными команды, содержащий:
            - linear (float/int)
            - angular (float/int)
            - experts (list)
            - task_completed (bool)
    
    Returns:
        dict: Ответ сервера или None в случае ошибки
    """
    try:
        url = "http://193.XX.XX.XX:8011/command/"
        headers = {"Content-Type": "application/json"}
        
        # Формируем тело запроса
        payload = {
            "command": command
        }
        
        response = requests.post(url, 
                               headers=headers,
                               data=json.dumps(payload))
        
        if response.status_code == 200 or response.status_code == 201:
            return response.json()
        else:
            print(f"Ошибка при добавлении команды: {response.status_code}")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"Ошибка при выполнении запроса: {e}")
        return None
    
def clear_history():

    try:
        url = "http://193.XX.XX.XX:8011/records/all"
        
        response = requests.delete(url)
        
        if response.status_code == 200 or response.status_code == 201:
            return response.json()
        else:
            print(f"Ошибка при удалении команд: {response.status_code}")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"Ошибка при выполнении запроса: {e}")
        return None

def create_record(
    base_url: str,
    linear: float,
    angular: float,
    experts: List[int],
    task_completed: bool = False,
    find_goal: bool = True
) -> Optional[Dict]:
    """
    Создает новую запись через API.
    
    Args:
        base_url (str): Базовый URL API (например, 'http://localhost:8011')
        linear (float): Линейное значение
        angular (float): Угловое значение
        experts (List[int]): Список оценок экспертов
        task_completed (bool): Флаг завершения задачи
        find_goal (bool): Флаг поиска цели
    
    Returns:
        Optional[Dict]: Ответ от сервера в виде словаря или None в случае ошибки
    
    Raises:
        requests.exceptions.RequestException: В случае ошибки при выполнении запроса
    """
    try:
        # Формируем URL для запроса
        endpoint = f"{base_url.rstrip('/')}/records/"
        
        # Формируем тело запроса
        payload = {
            "command": {
                "linear": linear,
                "angular": angular,
                "experts": experts,
                "task_completed": task_completed
            },
            "find_goal": find_goal
        }
        
        # Выполняем POST запрос
        response = requests.post(
            url=endpoint,
            json=payload,
            headers={"Content-Type": "application/json"}
        )
        
        # Проверяем статус ответа
        response.raise_for_status()
        
        # Возвращаем результат в виде словаря
        return response.json()
    
    except requests.exceptions.RequestException as e:
        print(f"Ошибка при выполнении запроса: {e}")
        return None


def upload_file(file_path: str) -> Dict[str, Any]:
    """
    Загружает файл на сервер через POST запрос.
    
    Args:
        file_path (str): Путь к файлу, который нужно загрузить
        
    Returns:
        Dict[str, Any]: Ответ сервера в формате JSON
        
    Raises:
        FileNotFoundError: Если файл не найден
        requests.RequestException: При ошибке отправки запроса
    """
    # Проверяем существование файла
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"Файл не найден: {file_path}")

    # Формируем полный URL
    url = f"{API_CONFIG['BASE_URL']}{API_CONFIG['UPLOAD_ENDPOINT']}"
    
    try:
        # Открываем файл в бинарном режиме
        with open(file_path, 'rb') as file:
            # Подготавливаем файл для отправки
            files = {
                'file': (os.path.basename(file_path), file, 'multipart/form-data')
            }
            
            # Отправляем POST запрос
            response = requests.post(
                url=url,
                headers=API_CONFIG['HEADERS'],
                files=files
            )
            
            # Проверяем статус ответа
            response.raise_for_status()
            
            # Возвращаем JSON ответ
            response = response.json()
            url = str(response.get('url', {}))
            url = url.replace("minio", "193.109.69.78")
            return url
            
    except requests.RequestException as e:
        # Логируем ошибку и пробрасываем её дальше
        print(f"Ошибка при загрузке файла: {str(e)}")
        raise

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
    


def get_numbers(command_dict):
    linear = int(command_dict['linear'])
    angular = int(command_dict['angular'])
    return linear, angular

import re
import json

def extract_command(api_response):
    # Получаем строку message из ответа
    text = api_response['message']
    # Паттерн для поиска JSON-подобной структуры, начинающейся с {"linear"
    pattern = r'{"linear":[^}]+}'
    
    # Поиск команды в тексте
    match = re.search(pattern, text)
    
    if match:
        command_str = match.group(0)
        try:
            # Пробуем распарсить команду как JSON
            command_dict = json.loads(command_str)
            return command_dict
        except json.JSONDecodeError:
            return None
    return None

def gpt_api(image_link, context):

    assistents = {"sonet": "ai2959328", "4o-mini": "ai3160570", "sonet_new": "ai8489712", "sonet_new_ai": "ai9530956"}
    try:    
        # random string 24 symbols
        random_string = ''.join(random.choices(string.ascii_uppercase + string.digits, k=24))
        
        payload = {
            "chatId": random_string,
            "assistantCode": assistents["sonet_new_ai"],
            "message": json.dumps(context),
            "maxContext": 0,
            "images": [image_link]

        }
        
        headers = {
            'Content-Type': 'application/json',
            'Authorization': 'yourtoken'
        }
        
        response = requests.post('https://gptunnel.ru/v1/assistant/chat', headers=headers, json=payload)
        
        if response.status_code == 200:
            print("Success:", response.json())
        else:
            print("Error:", response.status_code, response.text)

        command = extract_command(response.json())
        print(command)
        return command

            
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

def main():
    clear_history()
    controller = RobotController()
    images_count = 0
    while not rospy.is_shutdown():
        try:
            # Создаем объект подписчика камеры
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

                image_path = '/home/kravtandr/robot/Scripts/camera_image.jpg'
                client_id = '71221600a19873c'
                print("Запрос к Minio...")
                # image_link = upload_to_imgur(image_path, client_id, "Simple upload", "This is a simple image upload in Imgur").get('data', {}).get('link')
                image_link = upload_file(image_path)
                # Или отправить в другое API
                print("Запрос к FastAPI...")
                context = get_all_commands()
                print("Запрос к API GPT...")
                try:
                    command = gpt_api(image_link, context)
                    # сохраняем полученный результат
                    result = add_command_to_history(command)
                    if result:
                        print("Команда успешно добавлена к истории:")

                    # Держим узел работающим
                    if command["angular"]>0:
                        duration = abs(command["angular"]/15)
                        controller.turn(duration=duration,direction='right')
                    elif command["angular"]<0:
                        duration = abs(command["angular"]/15)
                        controller.turn(duration=duration,direction='left')

                    if command["linear"]>0:
                        duration = abs(command["linear"]/30)
                        controller.move_forward(duration=duration)
                    elif command["linear"]<0:
                        duration = abs(command["linear"]/30)
                        controller.move_backward(duration=duration)

                except Exception as e:
                    print(e)

                
                # rospy.sleep(1)  # Пауза перед сохранением изображения

            
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass