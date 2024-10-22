export const RobotCommands = {
  move: 'move',
  rotate: 'rotate',
  stop: 'stop',
}

export const MessageTypes = {
  msg: 'msg', 
  odo: 'odo',
  com: 'com',
}

// type CommandT = 'move' | 'rotate' | 'stop';
export type CommandT = keyof typeof RobotCommands;

interface RobotCommandsParamsMapI {
  move: {
    dir: 'forward' | 'backward';
    // speed: number;
  };
  rotate: {
    dir: 'left' | 'right';
    // angle: number;
  };
  stop: null;
}

export type CommandParamsT<C extends CommandT> = RobotCommandsParamsMapI[C];

export type WSCommandMessageT <C extends CommandT> = {
  command: C;
  params: CommandParamsT<C>;
}

export type MessageTypeT = keyof typeof MessageTypes; 

interface MessageTypeMapI<T extends CommandT> {
  msg: string;
  odo: Object;
  com: WSCommandMessageT<T>
}

export type MessagePayloadT<T extends MessageTypeT> = MessageTypeMapI<CommandT>[T];

export type RawMessageT<T extends MessageTypeT = MessageTypeT> = {
  type: T;
  data: MessagePayloadT<T>;
}

export interface MessageT<T extends MessageTypeT = MessageTypeT> extends RawMessageT<T> {
  time: string;
}
