const roomName = 'testroom';

const input = document.querySelector('#chat-message-input');
const form = document.querySelector('form');
const btnInfo = document.querySelector('input#info');
const btnCmd = document.querySelector('input#command');
const btnOdo = document.querySelector('input#odometry');
const log = document.querySelector('#chat-log');

if (!(input && form && log && btnInfo && btnCmd && btnOdo)) {
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
  log.innerHTML += '<p>recieved:' + JSON.stringify(data) + '</p>';
};

ws.onclose = (e) => {
  console.warn('Chat socket closed unexpectedly');
};

input.focus();
input.addEventListener('keyup', (e) => {
  if (e.key === 'enter') {  // enter, return
    btnInfo.click();
  }
});


const randomChoice = (array) => array[Math.floor(Math.random() * array.length)];
const randomCommand = () => {
  const command = randomChoice(['rotate', 'move', 'stop']);
  const params = {};
  switch(command) {
    case 'rotate':
      params['dir'] = randomChoice(['left', 'right']);
      // params['deg'] = Math.floor(Math.random() * 180);
      // выполняется, пока не стоп
    break;
    case 'move':
      params['dir'] = randomChoice(['forward', 'backward']);
      // params['speed'] = Math.floor(Math.random() * 255);
      // выполняется, пока не стоп
    break;
    case 'stop':
      // прервать предыдущую команду
      // params['when'] = randomChoice(['immidiate', Math.floor(Math.random() * 5)]);
      // params['when'] = randomChoice(['immidiate', Math.floor(Math.random() * 5)]);
    break;
    default:
      return {
        command: 'помехи',
        params,
      };
  }
  return {
    command,
    params,
  };
}

btnOdo.addEventListener('click', (e) => {
  e.preventDefault();
  const data = {
    speed: Math.floor(Math.random() * 1000) + 'sm/s',
    l_wheel_v: Math.floor(Math.random() * 1000) + 'rad/s',
    r_wheel_v: Math.floor(Math.random() * 1000) + 'rad/s',
  }
  const odometry = {
    type: 'odo',
    data,
  }
  log.innerHTML += `<p>[${Date.now().toString()}] sent odometry: ` + data.speed + '|' + data.l_wheel_v + '|' + data.r_wheel_v + '</p>';
  ws.send(JSON.stringify(odometry));
});

btnInfo.addEventListener('click', (e) => {
  e.preventDefault();
  log.innerHTML += `<p>[${Date.now().toString()}] sent info: ` + input.value + '</p>';
  ws.send(JSON.stringify({
    'type': 'msg',
    'data': input.value
  }));
  input.value = '';
});

btnCmd.addEventListener('click', (e) => {
  e.preventDefault();
  const command = {
    type: 'com',
    data: randomCommand(),
  }
  log.innerHTML += `<p>[${Date.now().toString()}] sent command: ` + command.data.command + JSON.stringify(command.data.params).slice(15) + '...</p>';
  ws.send(JSON.stringify(command));
});
