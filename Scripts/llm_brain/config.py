

class Config():
    API_CONFIG = {
        'BASE_URL': 'http://193.XXX.XX.XX:8011',
        'UPLOAD_ENDPOINT': '/api/v1/upload',
        'HEADERS': {
            'accept': 'application/json'
        }
    }
    
    ASSISTENTS = {
        "sonet": "ai2959328", 
        "4o-mini": "ai3160570", 
        "sonet_new": "ai8489712", 
        "sonet_new_ai": "ai9530956",
        "BBOT_SONET_NEW":"ai5583415",
        "BUSSINESS_SONET_NEW": "ai4798924",
        "BBOT_1":"ai9988896"
        }
    
    CHOOSED_ASSISTENT = ASSISTENTS["BBOT_1"]

    GPTTUNNEL_AUTH_TOKEN = "YOUR_TOKEN"

    IMGUR_CLIENT_ID = 'client_id'


