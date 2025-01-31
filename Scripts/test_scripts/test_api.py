import requests
import json


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
        url = f"http://193.XXX.XX.XX:8011/commands/?skip={skip}&limit={limit}"
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
        url = "http://193.XXX.XX.XX:8011/command/"
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
        url = "http://193.XXX.XX.XX:8011/records/all"
        
        response = requests.delete(url)
        
        if response.status_code == 200 or response.status_code == 201:
            return response.json()
        else:
            print(f"Ошибка при удалении команд: {response.status_code}")
            return None
            
    except requests.exceptions.RequestException as e:
        print(f"Ошибка при выполнении запроса: {e}")
        return None

# Получение списка команд
# commands = get_all_commands(skip=0, limit=10)
# if commands:
#     print("Полученные команды:", commands)

# Добавление новой команды
new_command = {
    "linear": 30,
    "angular": 15,
    "experts": [85, 90, 75],
    "task_completed": False
}

# result = add_command_to_history(new_command)
# if result:
#     print("Команда успешно добавлена:", result)


clear_history()
# Получение списка команд
commands = get_all_commands(skip=0, limit=10)
if commands:
    print("Полученные команды:", json.dumps(commands))