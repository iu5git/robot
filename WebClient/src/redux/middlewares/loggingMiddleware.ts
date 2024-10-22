import { Middleware } from 'redux'

import { RootStateT } from '../store'

const loggingMiddleware: Middleware<
  {}, // Most middleware do not modify the dispatch return value
  RootStateT
> = storeApi => next => action => {
  const store = storeApi.getState() // correctly typed as RootState
  console.log(store);
  next(action);
};

export default loggingMiddleware;
