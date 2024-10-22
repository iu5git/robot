import { FC } from "react";
import WS, {WSMaxRetries, WSRetryPlainTime} from '../../api/webSocket';
import ConnectButtonComponent from './connectButtonComponent';
import Spinner from '../spinner/spinnerComponent';

import {useSelector} from "react-redux";
import {selectTicks, selectStatus} from '../../redux/reducers/wsReducer';
import type {WSStatusT} from '../../redux/reducers/wsReducer';
import type {SpinnerClassesT} from '../spinner/spinnerComponent';

import './wsNotificator.scss'

interface WSNotifProps {
  props?: any,
}

const statusSpinnerClassMap: {
  [key in WSStatusT]: SpinnerClassesT
} = {
  "connected": 'online',
  "disconnected": 'offline',
  "reconnecting": 'loading',
  "connecting": 'loading',
  "error": 'error',
}

const statusTranslationMap: {
  [key in WSStatusT]: string
} = {
  "connected": 'Онлайн',
  "disconnected": 'Оффлайн',
  "reconnecting": 'Переподключение',
  "connecting": 'Подключение',
  "error": 'Ошибка',
}

const WSNotificationComponent: FC = () => {
  const status = useSelector(selectStatus);
  const count = useSelector(selectTicks);
  const timer = WSRetryPlainTime.round({smallestUnit: 'second', roundingMode: 'ceil'}).seconds - count;

  let button = <div></div>;
  if (status === 'disconnected' || status === 'error') {
    button = <ConnectButtonComponent text="Подключиться" onClick={() => WS.init()}/>
  } else if (status === 'connected') {
    button = <ConnectButtonComponent text="Отключиться" onClick={() => WS.close()}/>
  } else {
    button = <ConnectButtonComponent text="Сброс" onClick={() => WS.stop()}/>
  }

  return (
    <div className="ws_notif">
      <div className="ws_notif__heading">
          Соединение
        </div>
      <div className="ws_notif__container">
        <div className="ws_notif__row">
          <Spinner status={statusSpinnerClassMap[status]}/>
        </div>
        <div className="ws_notif__row">
          <span className={`ws_notif__status ${statusSpinnerClassMap[status]}`}>
            {statusTranslationMap[status]}
          </span>
        </div>
        <div className="ws_notif__row__message">
          <span>
            {status === 'reconnecting' ? timer ? ('через ' + timer) : 'сейчас' : ''}
            {status === 'error' ? 'Сервер недоступен' : ''}
          </span>
        </div>
        <div className="ws_notif__row__message">
          <span>
            {/* просклонять если надо */}
            {status === 'error' ? `после ${WSMaxRetries} попыток` : ''}
          </span>
        </div>
      </div>
      {button}
    </div>
)};

export default WSNotificationComponent;
