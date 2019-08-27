import { combineReducers } from 'redux'
import { routerReducer } from 'react-router-redux'

import { SERVICE_NAMES, services, auth } from '../feathers'
import modals from './modals-reducer'

let reducers = {
  modals,
  auth: auth.reducer,
  routing: routerReducer
}

// Add all service reducers
for (let i = 0; i < SERVICE_NAMES.length; i++) {
  let name = SERVICE_NAMES[i]
  reducers[name] = services[name].reducer
}

// Combine and export
const rootReducer = combineReducers(reducers)
export default rootReducer
