import csv
import os

class CSV_Logs:
    def __init__(self, file_path):
        self.file_path = file_path
        self.fieldnames = ['id', 'chatId', 'assistantId', 'assistantCode', 'message',
                          'model', 'spendTokenCount', 'historyMessageCount',
                          'context_messages', 'prompt_tokens', 'completion_tokens',
                          'embedding_tokens', 'total_tokens', 'prompt_cost',
                          'completion_cost', 'embedding_cost', 'total_cost']

    def prepare_data(self, data):
        """Подготовка данных для записи в CSV"""
        prepared_data = {
            'id': data.get('id', ''),
            'chatId': data.get('chatId', ''),
            'assistantId': data.get('assistantId', ''),
            'assistantCode': data.get('assistantCode', ''),
            'message': data.get('message', ''),
            'model': data.get('model', ''),
            'spendTokenCount': data.get('spendTokenCount', ''),
            'historyMessageCount': data.get('historyMessageCount', ''),
            'context_messages': data.get('usage', {}).get('context_messages', ''),
            'prompt_tokens': data.get('usage', {}).get('prompt_tokens', ''),
            'completion_tokens': data.get('usage', {}).get('completion_tokens', ''),
            'embedding_tokens': data.get('usage', {}).get('embedding_tokens', ''),
            'total_tokens': data.get('usage', {}).get('total_tokens', ''),
            'prompt_cost': data.get('usage', {}).get('prompt_cost', ''),
            'completion_cost': data.get('usage', {}).get('completion_cost', ''),
            'embedding_cost': data.get('usage', {}).get('embedding_cost', ''),
            'total_cost': data.get('usage', {}).get('total_cost', '')
        }
        return prepared_data

    def write_to_csv(self, data):
        """Записывает данные в CSV файл"""
        try:
            prepared_data = self.prepare_data(data)
            write_header = not os.path.exists(self.file_path)
            
            with open(self.file_path, 'a', newline='', encoding='utf-8') as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=self.fieldnames)
                
                if write_header:
                    writer.writeheader()
                
                writer.writerow(prepared_data)
            print("Successfully wrote logs to CSV")
        except Exception as e:
            print(f"Error writing to CSV: {e}")