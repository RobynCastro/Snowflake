import reduxThunk from 'redux-thunk'
import reduxPromiseMiddleware from 'redux-promise-middleware'
import reduxMulti from 'redux-multi'
import { routerMiddleware } from 'react-router-redux'
import { browserHistory } from 'react-router'
import logger from './logger'
import snapshot from './snapshot'

export default [
  reduxThunk,
  reduxMulti,
  reduxPromiseMiddleware(),
  routerMiddleware(browserHistory),
  logger,
  snapshot
]
