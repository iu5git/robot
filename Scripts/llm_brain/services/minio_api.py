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

class MinioAPI:

    def upload_file(self, file_path: str) -> Dict[str, Any]:
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
        url = f"{Config.API_CONFIG['BASE_URL']}{Config.API_CONFIG['UPLOAD_ENDPOINT']}"
        
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
                    headers=Config.API_CONFIG['HEADERS'],
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

