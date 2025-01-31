from PIL import Image


class ImageMerger:
    def __init__(self, save_path):
        self.save_path = save_path

    def merge_images_horizontal_scaled(self, image1_path, image2_path, output_path, target_height=None):
        # Открываем изображения
        img1 = Image.open(image1_path)
        img2 = Image.open(image2_path)

        img2 = img2.rotate(-90, expand=True)  # используем -90 для поворота по часовой стрелке

        # Отражаем второе изображение по вертикали
        img2 = img2.transpose(Image.FLIP_LEFT_RIGHT)
        
        # Получаем размеры
        width1, height1 = img1.size
        width2, height2 = img2.size
        
        # Определяем целевую высоту
        if target_height is None:
            target_height = min(height1, height2)
        
        # Масштабируем изображения, сохраняя пропорции
        ratio1 = target_height / height1
        ratio2 = target_height / height2
        
        new_width1 = int(width1 * ratio1)
        new_width2 = int(width2 * ratio2)
        
        # Простое изменение размера
        img1 = img1.resize((new_width1, target_height))
        img2 = img2.resize((new_width2, target_height))
        
        # Создаем новое изображение
        new_img = Image.new('RGB', (new_width1 + new_width2, target_height))
        
        # Вставляем изображения
        new_img.paste(img1, (0, 0))
        new_img.paste(img2, (new_width1, 0))
        
        # Сохраняем результат
        new_img.save(self.save_path + output_path)
