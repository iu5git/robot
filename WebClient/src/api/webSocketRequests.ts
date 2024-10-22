import WS from './webSocket';
import type {RawMessageT, CommandT, CommandParamsT} from './messageTypes';

function messageMsg(msg: string): RawMessageT<'msg'> {
  return {
    type: 'msg',
    data: msg,
  }
}

function message(msg: string) {
  WS.send(messageMsg(msg));
}

function odometryMsg(obj: Object): RawMessageT<'odo'> {
  return {
    type: 'odo',
    data: obj,
  }
}

function odometry(obj: Object) {
  WS.send(odometryMsg(obj));
}

function commandMsg<C extends CommandT>(type: C, params: CommandParamsT<C>): RawMessageT<'com'> {
  return {
    type: 'com',
    data: {
      command: type,  // TODO: научиться переводить названия типов в строки
      params,
    },
  }
}

const WSSend = {
  command: {
    move(direction: 'forward' | 'backward') {
      WS.send(commandMsg<'move'>('move', {dir: direction}));
    },
    rotate(direction: 'left' | 'right') {
      WS.send(commandMsg<"rotate">('rotate', {dir: direction}));
    },
    stop() {
      WS.send(commandMsg<"stop">('stop', null));
    },
  },
  message,
  odometry,
};

export default WSSend;
