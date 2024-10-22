import { FC } from "react";
import './logComponent.scss';

import type {MessageT} from '../../api/messageTypes'

interface logProps {
  messages: MessageT[],
}

const shortFullTypeNameMap = {
  'odo': 'METRIC',
  'com': 'COMMAND',
  'msg': 'INFO',
}

const shortNameStyleMap = {
  'odo': {
    color: 'green',
    fontWeight: 'bold',
  },
  'com': {
    color: 'black',
    fontWeight: 'bold',
  },
  'msg': {
    color: 'blue',
    fontWeight: 'bold',
  },
};

const commandNameMap = {
  'rotate': 'ROTATE',
  'move': 'MOVE',
  'stop': 'STOP',
}

function getComOdoPayloadObj(data: MessageT<'com' | 'odo'>['data']): {[key: string]: any} | null {
  if ('params' in data) {
    return typeof data.params === 'object' ? data.params : {};
  }
  return data;
}

function getComOdoPayloadStr(data: MessageT<'com' | 'odo'>['data']): string {
  const obj = getComOdoPayloadObj(data);
  let res: string = '';
  let notfirst: boolean = false;

  if ('command' in data) {
    res = `<span class=log__command>${commandNameMap[data['command']]}</span>&nbsp`;
  }

  for (const key in obj) {
    if (notfirst) {
      res += ' | ';
    }
    res += `${key}: ${obj[key]}`;
    notfirst = true;
  }
  return res;
}

function getMsgItself(msg: MessageT): string {
  let res: string = '';
  switch(msg.type) {
    case 'com':
      res = getComOdoPayloadStr(msg.data);
      break;
    case 'odo':
      res = getComOdoPayloadStr(msg.data);
      break;
    case 'msg':
      res = msg.data as MessageT<'msg'>["data"];
      break;
  }
  return res;
}

const LogComponent: FC<logProps> = (props) => {
  return (
    <div className="log">
      {props.messages.map((el, i) => {
        const time = el.time.toString();
        const type = shortFullTypeNameMap[el.type];
        const msg = getMsgItself(el);
        return(
          <p key={i}>
            <span style={{color: 'blue'}}>
              {'[' + time + ']'}
            </span>
            &nbsp;
            <span style={shortNameStyleMap[el.type]}>
              {type}
            </span>
            &nbsp;
            {/* {msg} */}
            <span dangerouslySetInnerHTML={{__html: msg}}></span>
          </p>
        )
      })}
    </div>
  );
};

export default LogComponent;
