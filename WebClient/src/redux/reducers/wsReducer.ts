import {Temporal} from '@js-temporal/polyfill';
import { createSlice, PayloadAction } from '@reduxjs/toolkit';
import type { RootStateT } from '../store';
import type { MessageTypeT, MessageT, RawMessageT } from '../../api/messageTypes'

export const WSStatus = {
  connected: 'connected',
  disconnected: 'disconnected',
  reconnecting: 'reconnecting',
  connecting: 'connecting',
  error: 'error',
}

export type WSStatusT = keyof(typeof WSStatus);

// odometry | server processing status | user command through this web client or platform

interface WSStateT {
  tick: number,
  status: WSStatusT,
  messages: (MessageT<'msg'> | MessageT<'com'>)[],
  odometry: MessageT<'odo'>[],
}


const initialState: WSStateT = {
  tick: 0,
  status: 'disconnected',
  messages: [],
  odometry: [],
}


function timeISORus() {
  return Temporal.Now.plainTimeISO().toLocaleString();
}


export const WSSlice = createSlice({
  name: 'ws',
  initialState,
  reducers: {
    start_connecting: (state) => {
      state.status = 'connecting';
    },
    connect: (state) => {
      state.status = 'connected';
    },
    disconnect: (state) => {
      state.status = 'disconnected';
    },
    reconnect: (state) => {
      state.status = 'reconnecting';
      state.tick = 0;
    },
    retriesLimit: (state) => {
      state.status = 'error';
    },
    tick: (state) => {
      state.tick++;
    },
    message: (state, action: PayloadAction<RawMessageT<MessageTypeT>>) => {
      switch(action.payload.type) {
        case 'odo': {
            const data = (action.payload as RawMessageT<'odo'>).data;
            console.log('[WS-Reducer]: got', action.payload.type, ' msg:', data);
            state.odometry.push({
              ...action.payload as RawMessageT<'odo'>,
              time: timeISORus(),
            });
          }
          break;
        case 'msg': {
            const data = (action.payload as RawMessageT<'msg'>).data;
            console.log('[WS-Reducer]: got', action.payload.type, ' msg:', data);
            state.messages.push({
              ...action.payload as RawMessageT<'msg'>,
              time: timeISORus(),
            });
          }
          break;
        case 'com': {
            const data = (action.payload as RawMessageT<'com'>).data;
            console.log('[WS-Reducer]: got', action.payload.type, ' msg:', data);
            state.messages.push({
              ...action.payload as RawMessageT<'com'>,
              time: timeISORus(),
            });
          }
          break;
        default:
          console.warn('[WS-Reducer]: unsupported message format', action.payload);
      }
    }
  }
})

export const WSActions = WSSlice.actions;
export const selectTicks = (state: RootStateT) => state.WSReducer.tick;
export const selectStatus = (state: RootStateT) => state.WSReducer.status;
export const selectMessages = (state: RootStateT) => state.WSReducer.messages;
export const selectOdometry = (state: RootStateT) => state.WSReducer.odometry;

export default WSSlice.reducer
