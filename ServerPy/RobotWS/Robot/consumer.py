# chat/consumers.py
from copyreg import constructor
import json
from channels.generic.websocket import AsyncWebsocketConsumer


class RobotConsumer(AsyncWebsocketConsumer):
    def constructor(self):
        print('RobotConsumer ctor')
        for prop in self:
            print(prop, self[prop])

    async def connect(self):
        print('RobotConsumer connect')
        self.room_name = self.scope['url_route']['kwargs']['room_name']
        self.room_group_name = 'chat_%s' % self.room_name

        # Join room group
        await self.channel_layer.group_add(
            self.room_group_name,
            self.channel_name
        )

        await self.accept()

    async def disconnect(self, close_code):
        print('RobotConsumer disconnect')
        # Leave room group
        await self.channel_layer.group_discard(
            self.room_group_name,
            self.channel_name
        )

    # Receive message from WebSocket
    async def receive(self, text_data):
        text_data_json = json.loads(text_data)
        # message = text_data_json['message']
        message = text_data_json
        print('RobotConsumer recieved', text_data)

        # Send message to room group
        await self.channel_layer.group_send(
            self.room_group_name,
            {
                'type': 'topic_message',
                'message': message
            }
            # text_data_json
        )

    # Receive message from room group
    async def topic_message(self, event):
        message = event['message']
        # message = event
        print('RobotConsumer recieved topic', message)

        # Send message to WebSocket
        # await self.send(text_data=json.dumps({
        #     'message': message
        # }))
        await self.send(text_data=json.dumps(message))
