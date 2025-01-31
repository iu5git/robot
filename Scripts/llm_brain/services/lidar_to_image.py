#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import os
from datetime import datetime
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge


class LidarToImage:
    """
    Класс для преобразования данных лидара в изображение.
    """
    
    def __init__(self, save_path="lidar_images"):
        """
        Инициализация узла ROS и подписка на топик лидара.
        
        Args:
            save_path (str): путь для сохранения изображений
        """
        # Инициализация узла ROS
        # rospy.init_node('lidar_to_image', anonymous=True)
        # Ориентация робота (в радианах)
        self.robot_orientation = np.pi  # разворот на 180 градусов
        # Параметры изображения
        self.image_size = 382
        self.scale_factor = 50
        self.center = self.image_size // 2
        
        # Параметры сохранения
        self.save_path = save_path
        self.ensure_save_directory()
        
        # Создание моста для конвертации изображений
        self.bridge = CvBridge()
        
        # Подписка на топик лидара
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # Создание пустого изображения
        self.image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        
        # Флаг для автоматического сохранения
        # self.auto_save = False
        # self.save_interval = rospy.Duration(1.0)  # интервал автосохранения (1 секунда)
        # self.last_save_time = rospy.Time.now()

                # Параметры камеры
        self.camera_fov = 120  # угол обзора камеры в градусах
        self.camera_range = 3.0  # дальность отрисовки области обзора камеры в метрах
        self.camera_color = (0, 165, 255)  # оранжевый цвет для области обзора камеры
        
        # Вычисляем углы для отрисовки области обзора камеры
        self.camera_half_fov = np.deg2rad(self.camera_fov / 2)

    def ensure_save_directory(self):
        """
        Создание директории для сохранения изображений, если она не существует.
        """
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
            rospy.loginfo(f"Создана директория для сохранения: {self.save_path}")

    def generate_filename(self, prefix="lidar", itteration=0):
        """
        Генерация имени файла для сохранения изображения.
        
        Args:
            prefix (str): префикс имени файла
            
        Returns:
            str: полное имя файла
        """
        timestamp = datetime.now().strftime("%d_%m_%Y_%H-%M-%S_%f")
        return os.path.join(self.save_path, f"{itteration}_{prefix}_{timestamp}.png")

    def save_image(self, filename=None, itteration=0):
        """
        Сохранение текущего изображения в файл.
        
        Args:
            filename (str, optional): имя файла для сохранения
            
        Returns:
            str: путь к сохраненному файлу
        """
        if self.image is None:
            rospy.logwarn("Нет доступного изображения для сохранения")
            return None

        if filename is None:
            filename = self.generate_filename(itteration=itteration)

        try:
            cv2.imwrite(filename, self.image)
            rospy.loginfo(f"Изображение сохранено: {filename}")
            return filename
        except Exception as e:
            rospy.logerr(f"Ошибка при сохранении изображения: {e}")
            return None

    def set_auto_save(self, enable=True, interval=1.0):
        """
        Включение/выключение автоматического сохранения изображений.
        
        Args:
            enable (bool): включить/выключить автосохранение
            interval (float): интервал сохранения в секундах
        """
        self.auto_save = enable
        self.save_interval = rospy.Duration(interval)
        rospy.loginfo(f"Автосохранение {'включено' if enable else 'выключено'} с интервалом {interval} сек")

    def polar_to_cartesian(self, range_data, angle_data):
        """
        Преобразование полярных координат в декартовы.
        
        Args:
            range_data (float): расстояние от лидара до точки
            angle_data (float): угол измерения в радианах
            
        Returns:
            tuple: координаты x, y в декартовой системе
        """
        x = range_data * np.cos(angle_data)
        y = range_data * np.sin(angle_data)
        return x, y

    def create_image_from_scan(self, scan_msg):
        """
        Создание изображения из данных сканирования.
        
        Args:
            scan_msg: сообщение типа LaserScan
            
        Returns:
            numpy.ndarray: изображение в формате OpenCV
        """
        # Создание чистого изображения
        image = np.zeros((self.image_size, self.image_size, 3), dtype=np.uint8)
        
        # Получение углов для каждого измерения
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment, 
                          scan_msg.angle_increment)
        
        # Преобразование каждой точки
        for range_value, angle in zip(scan_msg.ranges, angles):
            # Пропуск невалидных значений
            if range_value < scan_msg.range_min or range_value > scan_msg.range_max:
                continue
                
            # Преобразование в декартовы координаты
            x, y = self.polar_to_cartesian(range_value, angle)
            
            # Масштабирование и смещение для отображения на изображении
            pixel_x = int(x * self.scale_factor) + self.center
            pixel_y = int(y * self.scale_factor) + self.center
            
            # Проверка, что точка находится в пределах изображения
            if 0 <= pixel_x < self.image_size and 0 <= pixel_y < self.image_size:
                # Рисование точки
                cv2.circle(image, (pixel_x, pixel_y), 2, (0, 255, 0), -1)

        # Отрисовка области обзора камеры
        image = self.draw_camera_fov(image)
        
        # Рисование позиции робота
        cv2.circle(image, (self.center, self.center), 8, (255, 0, 255), -1)
        
        return image

    def scan_callback(self, scan_msg):
        """
        Callback-функция для обработки данных с лидара.
        
        Args:
            scan_msg: сообщение типа LaserScan
        """
        # Создание изображения из данных сканирования
        self.image = self.create_image_from_scan(scan_msg)
        
        # Отображение изображения (для отладки)
        # cv2.imshow('Lidar Visualization', self.image)
        # cv2.waitKey(1)

    def get_current_image(self):
        """
        Получение текущего изображения.
        
        Returns:
            numpy.ndarray: текущее изображение
        """
        return self.image.copy()
    
    def draw_camera_fov(self, image):
        """
        Отрисовка области обзора камеры на изображении.
        
        Args:
            image (numpy.ndarray): изображение для отрисовки
            
        Returns:
            numpy.ndarray: изображение с отрисованной областью обзора
        """
        # Вычисляем точки для отрисовки области обзора
        # Инвертируем углы для правильного отображения на изображении
        start_angle = self.robot_orientation + self.camera_half_fov # левая граница
        end_angle = self.robot_orientation - self.camera_half_fov  # правая граница
        
        # Координаты начала области (позиция робота)
        start_point = (self.center, self.center)
        
        # Вычисляем конечные точки линий области обзора
        left_x = int(self.center + np.cos(start_angle) * self.camera_range * self.scale_factor)
        left_y = int(self.center + np.sin(start_angle) * self.camera_range * self.scale_factor)
        right_x = int(self.center + np.cos(end_angle) * self.camera_range * self.scale_factor)
        right_y = int(self.center + np.sin(end_angle) * self.camera_range * self.scale_factor)
        
        # Отрисовка границ области обзора
        cv2.line(image, start_point, (left_x, left_y), self.camera_color, 2)
        cv2.line(image, start_point, (right_x, right_y), self.camera_color, 2)
        
        # Отрисовка дуги
        # Преобразуем углы в градусы для OpenCV
        start_angle_deg = int(np.rad2deg(start_angle))
        end_angle_deg = int(np.rad2deg(end_angle) )
        
        # Убедимся, что start_angle_deg меньше end_angle_deg
        if start_angle_deg > end_angle_deg:
            start_angle_deg, end_angle_deg = end_angle_deg, start_angle_deg
        
        # Радиус дуги в пикселях
        radius = int(self.camera_range * self.scale_factor)
        
        # Рисуем дугу
        cv2.ellipse(image, start_point, (radius, radius), 
                   0, min(start_angle_deg, end_angle_deg), 
                   max(start_angle_deg, end_angle_deg), 
                   self.camera_color, 2)
    
        # Добавляем индикатор направления (стрелку)
        # direction_x = int(self.center + np.cos(self.robot_orientation) * 20)
        # direction_y = int(self.center + np.sin(self.robot_orientation) * 20)
        # cv2.arrowedLine(image, start_point, (direction_x, direction_y), 
        #                (0, 178, 255), 2, tipLength=0.3)
        
        return image

    def set_robot_orientation(self, angle_degrees):
        """
        Установка ориентации робота.
        
        Args:
            angle_degrees (float): угол в градусах (0 - вправо, 90 - вверх)
        """
        self.robot_orientation = np.deg2rad(angle_degrees)

# def main():
#     """
#     Основная функция для запуска узла.
#     """
#     try:
#         lidar_to_image = LidarToImage()
        
#         # Пример использования автосохранения
#         # lidar_to_image.set_auto_save(True, interval=2.0)  # Сохранение каждые 2 секунды
        
#         # Обработка клавиш для ручного сохранения
#         rate = rospy.Rate(10)  # 10 Hz
        
#         while not rospy.is_shutdown():
#             lidar_to_image.save_image()
            
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()