import csv

import csv
import json

class CSV_Logs:
    def __init__(self, file_path):
        self.file_path = file_path
        self.fieldnames = ["status", "id", "chatId", "assistantId", "assistantCode", 
                          "message", "model", "spendTokenCount", "historyMessageCount",
                          "context_messages", "prompt_tokens", "completion_tokens",
                          "embedding_tokens", "total_tokens", "prompt_cost",
                          "completion_cost", "embedding_cost", "total_cost"]

    def parse_data(self, data):
        """Парсит строку данных в словарь"""
        try:
            # Разделяем статус и JSON
            status, json_str = data.split(": ", 1)
            # Парсим JSON
            json_data = json.loads(json_str)
            
            # Формируем словарь с данными
            parsed_data = {
                "status": status,
                "id": json_data.get("id", ""),
                "chatId": json_data.get("chatId", ""),
                "assistantId": json_data.get("assistantId", ""),
                "assistantCode": json_data.get("assistantCode", ""),
                "message": json_data.get("message", ""),
                "model": json_data.get("model", ""),
                "spendTokenCount": json_data.get("spendTokenCount", ""),
                "historyMessageCount": json_data.get("historyMessageCount", ""),
                "context_messages": json_data.get("usage", {}).get("context_messages", ""),
                "prompt_tokens": json_data.get("usage", {}).get("prompt_tokens", ""),
                "completion_tokens": json_data.get("usage", {}).get("completion_tokens", ""),
                "embedding_tokens": json_data.get("usage", {}).get("embedding_tokens", ""),
                "total_tokens": json_data.get("usage", {}).get("total_tokens", ""),
                "prompt_cost": json_data.get("usage", {}).get("prompt_cost", ""),
                "completion_cost": json_data.get("usage", {}).get("completion_cost", ""),
                "embedding_cost": json_data.get("usage", {}).get("embedding_cost", ""),
                "total_cost": json_data.get("usage", {}).get("total_cost", "")
            }
            return parsed_data
        except Exception as e:
            print(f"Error parsing data: {e}")
            return None

    def write_to_csv(self, data):
        """Записывает данные в CSV файл"""
        try:
            parsed_data = self.parse_data(data)
            if parsed_data:
                write_header = not os.path.exists(self.file_path)
                
                with open(self.file_path, "a", newline="", encoding="utf-8") as csv_file:
                    writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
                    
                    if write_header:
                        writer.writeheader()
                    
                    writer.writerow(parsed_data)
                print("Successfully wrote logs to CSV")
            else:
                print("No data to write")
        except Exception as e:
            print(f"Error writing to CSV: {e}")


data = """Success: {"id": "674394ccbdfa1d0001ceb5b7", "chatId": "8OA8JWO1HWDHE9G9HRGPD4T6", "assistantId": "67437fa42fda9fe3c90bc4d6", "assistantCode": "ai4798924", "message": "Анализирую ситуацию:\n\nЦель не выслежена: Красная стрелка не видна в поле зрения\nМестность: Коридор-лабиринт серый\n\nВижу:\n- Нахожусь в начале пути\n- Впереди коридор с поворотом\n- Стены серые, пол зеленый\n- Видно небо впереди, что указывает на возможный выход\n\nПлан действий:\nДвигаться вперед для исследования пространства. Скорость умеренная для безопасности.\n\nCommand: {"linear": 50, "angular": 0, "experts": [90, 70, 100], "task_completed": false}\n\nОбоснование оценок экспертов:\n- Безопасность 90: Путь впереди чист\n- Поиск цели 70: Нужно исследовать пространство\n- Предыдущие решения 100: Первое решение", "model": "claude-3.5-sonnet", "spendTokenCount": 1966, "historyMessageCount": 2, "usage": {"context_messages": 2, "prompt_tokens": 1699, "completion_tokens": 267, "embedding_tokens": 0, "total_tokens": 1966, "prompt_cost": 2.5485, "completion_cost": 2.0025, "embedding_cost": 0, "total_cost": 4.551}}"""
# data = """Success: {"id": "674394ccbdfa1d0001ceb5b7", "chatId": "8OA8JWO1HWDHE9G9HRGPD4T6", "assistantId": "67437fa42fda9fe3c90bc4d6", "assistantCode": "ai4798924", "message": "Анализирую ситуацию...", "model": "claude-3.5-sonnet", "spendTokenCount": 1966, "historyMessageCount": 2, "usage": {"context_messages": 2, "prompt_tokens": 1699, "completion_tokens": 267, "embedding_tokens": 0, "total_tokens": 1966, "prompt_cost": 2.5485, "completion_cost": 2.0025, "embedding_cost": 0, "total_cost": 4.551}}"""
import os


# Создаем экземпляр класса
csv_logger = CSV_Logs("logs.csv")

# Записываем данные
csv_logger.write_to_csv(data)