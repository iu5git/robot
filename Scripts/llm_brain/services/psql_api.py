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
import requests
from typing import List, Dict, Union, Optional
from config import Config


class PostgresAPI:
    def __init__(self):
        pass

    def get_all_commands(self, skip=0, limit=100):
        """
        Получает список всех команд с пагинацией
        
        Args:
            skip (int): Количество пропускаемых записей
            limit (int): Максимальное количество возвращаемых записей
        
        Returns:
            list: Список команд или None в случае ошибки
        """
        try:
            url = f"{Config.API_CONFIG['BASE_URL']}/commands/?skip={skip}&limit={limit}"
            response = requests.get(url)
            
            if response.status_code == 200:
                return response.json()
            else:
                print(f"Ошибка при получении команд: {response.status_code}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"Ошибка при выполнении запроса: {e}")
            return None

    def add_command_to_history(self, command):
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
            url = f"{Config.API_CONFIG['BASE_URL']}/command/"
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
        
    def clear_history(self):

        try:
            url = f"{Config.API_CONFIG['BASE_URL']}/records/all"
            
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
        self, 
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



