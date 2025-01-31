import re
import json

def extract_command(text):
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

# Пример использования
text = '''Оценка: 75/100 - неисследованная область справа требует проверки

[Итоговое решение]:
Учитывая все рекомендации, выполняем безопасное движение с небольшим поворотом.
Command: {"linear": 30, "angular": 15, "experts": [85,90,75], "task_completed": false }
Средняя оценка уверенности: 83/100

Пример ответа при достижении цели:
Цель найдена: Да
Местность: Комната'''

command = extract_command(text)
if command:
    print("Найдена команда:")
    print(f"linear: {command['linear']}")
    print(f"angular: {command['angular']}")
    print(f"experts: {command['experts']}")
    print(f"task_completed: {command['task_completed']}")
else:
    print("Команда не найдена")