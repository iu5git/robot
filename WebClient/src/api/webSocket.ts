import {wsDebug} from '../base/globals';
import { Temporal } from "@js-temporal/polyfill";

import store from '../redux/store';
import { WSActions, selectStatus } from '../redux/reducers/wsReducer';

// const APIurl = 'ws://127.0.0.1/ws';
// Локальная разработка
const room = 'testroom';
const APIurl = `ws://localhost:8080/ws/robot/${room}/`;

let webSocket: null | WebSocket = null;

export const WSretryDelay = 5000;
export const WSMaxRetries = 3;
export const WSRetryPlainTime = Temporal.Duration.from({milliseconds: WSretryDelay});

let retriesCnt = 1;
let timerStopper: NodeJS.Timeout | null = null;
let intervalStopper: NodeJS.Timer | null = null;

export default class WS {
  /**
   * стартует WS соединение с сервером
   * @param {boolean} doRetry [= true]
   */
  static init(doRetry = true) {      
    const status = store.getState().WSReducer.status;
    // запрещаю создавать несколько соединений. Создание
    // подключения допустимо только при следующих статусах: 
    if (webSocket !== null || !(
        status === 'error' ||
        status === 'disconnected' ||
        status === 'reconnecting' ||
        false)) {
      return;
    }
    store.dispatch(WSActions.start_connecting());

    webSocket = new WebSocket(APIurl);

    webSocket.onopen = (e) => {
      if (wsDebug) {
        console.log('[WS]Соединение установлено');
      }
      store.dispatch(WSActions.connect());
    };

    webSocket.onmessage = (e: MessageEvent) => {
      if (wsDebug) {
        console.log('[WS]Данные получены с сервера:', e.data);
      }

      let data: any = {};
      try {
        data = JSON.parse(e.data);
      } catch (err: unknown) {
        console.warn('[WS]Ошибка:', (err as Error).message);
        return;
      }

      if (!('type' in data && 'data' in data)) {
        console.warn('[WS]Нарушен формат сообщений:', data);
        return;
      }

      store.dispatch(WSActions.connect());
      // логика тут
      console.log('[WS]Обработка данных...', data)
      store.dispatch(WSActions.message(data));
    };

    webSocket.onclose = (e: CloseEvent) => {
      if (e.wasClean) {
        if (wsDebug) {
          console.log(`[WS]Соединение закрыто чисто\n
          код: ${e.code}\nпричина: ${e.reason}`);
        }
        store.dispatch(WSActions.disconnect());
      } else {
        if (wsDebug) {
          console.warn(`[WS]Соединение прервано`);
        }

        if (doRetry) {
          if (retriesCnt > WSMaxRetries) {
            if (wsDebug) {
              console.log('[WS]Превышено число переподключений:', WSMaxRetries);
            }
            store.dispatch(WSActions.retriesLimit());
            this.close();
            retriesCnt = 1;
            return;
          }
          if (wsDebug) {
            console.log('[WS]Восстанавливаю соединение. Попытка №' + retriesCnt);
          }
          store.dispatch(WSActions.reconnect());
          intervalStopper = setInterval(() => store.dispatch(WSActions.tick()), 1000);
          timerStopper = setTimeout(() => {
            if (intervalStopper) {
              clearInterval(intervalStopper);
            }
            retriesCnt++;
            WS.close();
            WS.init();
          }, WSretryDelay);
        } else {
          store.dispatch(WSActions.disconnect());
        }
      }
    };

    webSocket.onerror = (e: Event) => {
      if (wsDebug) {
        console.warn(`[WS]Ошибка`);
      }
    };
  }

  static send(data: Object) {
    if (webSocket === null) {
      console.warn(`[WS]Невозможно отправить: соединение не установлено`);
      return;
    }
    webSocket.send(JSON.stringify(data));
  }

  /**
   * закрываем
   */
  static close() {
    if (webSocket) {
      webSocket.close();
      webSocket = null;
    }
  }

  static stop() {
    if (wsDebug) {
      console.log('[WS]Сброшено пользователем');
    }
    if (timerStopper) {
      clearTimeout(timerStopper);
    }
    if (intervalStopper) {
      clearInterval(intervalStopper);
    }
    timerStopper = null;
    intervalStopper = null;
    if (webSocket) {
      /*
        Обязательно снести коллбек!
        Они будут иметь разные значения переменных в своем замыкании, ведь
        при ретрае каждый коллбек создает новый объект на месте, где был this

      / проблема в том, что если закрыть соединение принудительно до
      / провала запроса на хендшейк, то onerror выполняется, а если после,
      / в ретрае, то сотрется, не изменив значения флагов и забагав счетчик

      */
      
      webSocket.onerror = null;
      webSocket.onclose = null;
    }
    this.close()
    store.dispatch(WSActions.disconnect());
    retriesCnt = 1;
  }
}
