import { bindActionCreators } from 'redux'
import { connect } from 'react-redux'

import store from '../store'
import client, { services } from '../feathers'
import actions from '../actions'

import Main from './main'

function stateToProps (state) {
  return {
    ...state,
    store,
    getStoreData: name => state[name].keys.map(key => state[name].store[key]),
    getData: name => state[name].store[state[name].keys[0]]
  }
}
function dispatchToProps (dispatch) {
  return {
    client,
    services,
    ...bindActionCreators(actions, dispatch)
  }
}

const App = connect(stateToProps, dispatchToProps)(Main)
export default App
