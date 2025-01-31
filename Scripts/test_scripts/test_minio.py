import os
import requests
from typing import Union, Dict, Any


API_CONFIG = {
    'BASE_URL': 'http://193.XXX.XX.XX:8011',
    'UPLOAD_ENDPOINT': '/api/v1/upload',
    'HEADERS': {
        'accept': 'application/json'
    }
}

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
            url = url.replace("minio", "193.XXX.XX.XX")
            return url
            
    except requests.RequestException as e:
        # Логируем ошибку и пробрасываем её дальше
        print(f"Ошибка при загрузке файла: {str(e)}")
        raise




