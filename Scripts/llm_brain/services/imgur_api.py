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
    
