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
from config import Config
import re
import json

class GPT_API:
    def __init__(self):
        pass

    def get_numbers(self, command_dict):
        linear = int(command_dict['linear'])
        angular = int(command_dict['angular'])
        return linear, angular



    def extract_command(self, api_response):
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

    def gpt_api(self, image_links, context):

        
        try:    
            # random string 24 symbols
            random_string = ''.join(random.choices(string.ascii_uppercase + string.digits, k=24))
            payload = {
                "chatId": random_string,
                "assistantCode": Config.CHOOSED_ASSISTENT,
                "message": "История команд: " + json.dumps(context),
                "images": image_links

            }
            print(payload)
            
            headers = {
                'Content-Type': 'application/json',
                'Authorization': Config.GPTTUNNEL_AUTH_TOKEN
            }
            response = requests.post('https://gptunnel.ru/v1/assistant/chat', headers=headers, json=payload)
            
            response = response.json()

            return response

                
        except Exception as e:
            print(f"An error occurred: {str(e)}")
