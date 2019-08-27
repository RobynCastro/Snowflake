import 'babel-polyfill'

import { createStore, compose, applyMiddleware } from 'redux'
import { syncHistoryWithStore } from 'react-router-redux'
import { browserHistory } from 'react-router'

// import { bindServices } from './feathers'
import middleware from './middleware'
import rootReducer from './reducers'

const initalState = {}

const REDUX_DEV = process.env.NODE_ENV !== 'production' && window.__REDUX_DEVTOOLS_EXTENSION__
const store = createStore(
  rootReducer,
  initalState,
  REDUX_DEV
    ? compose(applyMiddleware(...middleware), REDUX_DEV())
    : applyMiddleware(...middleware)
)

// bindServices(store)

export const history = syncHistoryWithStore(browserHistory, store)

// HMR
if (module.hot) {
  module.hot.accept('./reducers', () => {
    const nextReducers = require('./reducers').default
    store.replaceReducer(nextReducers)
  })
}

export default store
