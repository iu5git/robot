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
import os
import requests
from typing import Union, Dict, Any

def gpt_api(image_link):
    try:    
        payload = {
            "chatId": "111111111111111111111111111",
            "assistantCode": "ai2959328",
            "message": "Доберись до цели",
            "maxContext": 5,
            "images": [image_link]

        }
        
        headers = {
            'Content-Type': 'application/json',
            'Authorization': 'yourtoken'
        }
        print(payload)
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



print(gpt_api("/home/kravtandr/Downloads/tt.jpeg"))