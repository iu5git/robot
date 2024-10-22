const roomName = 'testroom';

const input = document.querySelector('#chat-message-input');
const submit = document.querySelector('#chat-message-submit');
const form = document.querySelector('form');
const log = document.querySelector('#chat-log');

if (!(input && submit && form && log)) {
  console.warn('no element');
}

const ws = new WebSocket(
  'ws://'
  + window.location.host
  + '/ws/robot/'
  + roomName
  + '/'
);

ws.onopen = (e) => {
  console.warn('ws connection established', ws);
}

ws.onmessage = (e) => {
  const data = JSON.parse(e.data);
  console.log(data);
  log.innerHTML += 'recieved:' + data.message + '\n';
};

ws.onclose = (e) => {
  console.warn('Chat socket closed unexpectedly');
};

input.focus();
input.addEventListener('keyup', (e) => {
  if (e.key === 'enter') {  // enter, return
    submit.click();
  }
});

form.addEventListener('submit', (e) => {
  e.preventDefault();
  log.innerHTML += 'sent: ' + input.value + '\n';
  ws.send(JSON.stringify({
    'message': input.value
  }));
  input.value = '';
});
