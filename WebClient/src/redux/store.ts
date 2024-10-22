import {combineReducers, AnyAction} from 'redux';
import { configureStore, PayloadAction } from '@reduxjs/toolkit';
import thunk, {ThunkAction} from 'redux-thunk';

import WSReducer from './reducers/wsReducer';
import { createLogger } from 'redux-logger';

// https://github.com/LogRocket/redux-logger#predicate--getstate-function-action-object--boolean
const loggingMiddleware = createLogger({
  predicate: (getState, action: PayloadAction) =>
    !(action.type === 'ws/tick' ||
      // action.type === 'ws/message' ||
      false
    ) 
});

export const store = configureStore({
  reducer: {
    WSReducer
  },
  middleware: (getDefaultMiddleware) =>
    getDefaultMiddleware()
      .prepend(loggingMiddleware, thunk),
});

export default store;

// Infer the `RootState` and `AppDispatch` types from the store itself
// export type RootState = ReturnType<typeof store.getState>
const rootReducer = combineReducers({ WSReducer });
export type RootStateT = ReturnType<typeof rootReducer>;
// Inferred type: {posts: PostsState, comments: CommentsState, users: UsersState}
export type AppDispatchT = typeof store.dispatch;
export type AppStateGetterT = typeof store.getState;
// Note that this assumes that there is no meaningful return value from the thunk.
// If your thunk returns a promise and you want to use the returned promise after dispatching the thunk,
// you'd want to use this as AppThunk<Promise<SomeReturnType>>
export type AppThunk<ReturnType = void> = ThunkAction<
  ReturnType,
  RootStateT,
  unknown,
  AnyAction
>
