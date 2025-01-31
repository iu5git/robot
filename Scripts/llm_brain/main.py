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

from  services.lidar_to_image import LidarToImage
from  services.bbot_controller import RobotController
from  services.camera_controller import CameraSubscriber
from  services.minio_api import MinioAPI
from  services.imgur_api import upload_to_imgur
from  services.psql_api import PostgresAPI
from  services.gpt_api import GPT_API
from  services.csv_utils import CSV_Logs
from config import Config
from utils.img_utils import ImageMerger



def main():
    route_dir ="routes/"
    timestamp = datetime.now().strftime("%d_%m_%Y_%H-%M-%S")
    route_dir +=os.path.join(f"route_{timestamp}")

    camera_sub = CameraSubscriber(save_path=route_dir)
    lidar_to_image = LidarToImage(save_path=route_dir)
    controller = RobotController()
    gpt_api = GPT_API()
    minio_api = MinioAPI()
    psql_api = PostgresAPI()
    psql_api.clear_history()
    csv_utils = CSV_Logs(os.path.join(route_dir, "logs.csv"))
    img_merger = ImageMerger(save_path=route_dir)

    rospy.sleep(3)

    images_count = 0
    while not rospy.is_shutdown():
        try:
            # Получаем изображение
            image = camera_sub.get_image()
            images_count +=1

            # Lidar image
            lidar_image_filepath = lidar_to_image.save_image(itteration=images_count)

            if image is not None:
                print("Изображение получено " + str(images_count) + "й раз")
               
                # Здесь можно выполнить любую обработку изображения
                # Например, сохранить его
                camera_image_path = camera_sub.save_image(itteration=images_count)

                image_links = []
                print("Запрос к Minio...")
                # camera_image_link = upload_to_imgur(image_path, Config.IMGUR_CLIENT_ID, "Simple upload", "This is a simple image upload in Imgur").get('data', {}).get('link')
                # lidar_image_link = upload_to_imgur(lidar_image_filepath, Config.IMGUR_CLIENT_ID, "Simple upload", "This is a simple image upload in Imgur").get('data', {}).get('link')
                camera_image_link = minio_api.upload_file(camera_image_path)
                lidar_image_link = minio_api.upload_file(lidar_image_filepath)


                merged_image_path = f"/{images_count}_merged.jpg"
                img_merger.merge_images_horizontal_scaled(camera_image_path, lidar_image_filepath, output_path=merged_image_path)
                merged_image_link = minio_api.upload_file(route_dir+ "/"+ merged_image_path)
                print("Запрос к Minio - OK")
                image_links.append(merged_image_link)
                # image_links.append(lidar_image_link)  

                
                
                # Или отправить в другое API
                print("Запрос к FastAPI...")
                context = psql_api.get_all_commands()
                print("Запрос к FastAPI - OK")

                print("Запрос к API GPT...")
                # rospy.sleep(100) 
                try:

                    response = gpt_api.gpt_api(image_links, context)
                    print("Ответ GPT:",response)
                    print(type(response))

                    print("Записываем полученный результат в CSV...")
                    csv_utils.write_to_csv(response)

                    
                    command = gpt_api.extract_command(response)
                    print("Получили команду:",command)


                    # сохраняем полученный результат
                    result = psql_api.add_command_to_history(command)
                    if result:
                        print("Команда успешно добавлена к истории:")

                    print("Запрос к API GPT - OK")
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

                
                # rospy.sleep(2)  # Пауза перед сохранением изображения

            
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass