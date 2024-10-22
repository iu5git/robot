import React from 'react';
import ReactDOM from 'react-dom';

import AppRouter from './base/router';

import store from './redux/store'
import { Provider } from 'react-redux'

import wrapConsoleLogWithColoredTags from './base/concoleColor';

// eslint-disable-next-line no-native-reassign
console = wrapConsoleLogWithColoredTags;

ReactDOM.render(
  <React.StrictMode>
    <Provider store={store}>
      <AppRouter />
      {/* import logo from './logo.svg';
      <img src={logo} className="App-logo" alt="logo" /> */}
    </Provider>
  </React.StrictMode>,
  document.getElementById('root')
);

// If you want to start measuring performance in your app, pass a function
// to log results (for example: reportWebVitals(console.log))
// or send to an analytics endpoint. Learn more: https://bit.ly/CRA-vitals
// reportWebVitals(console.log);
