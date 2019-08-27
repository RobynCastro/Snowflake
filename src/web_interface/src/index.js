import React from 'react'
import { render } from 'react-dom'
import { Provider } from 'react-redux'
import { Router } from 'react-router'

import store, { history } from './store'
import routes from './routes'

import './styles/visualizer.css'
import './styles/visualizer.css.map'

console.log(new Date().toLocaleString(), 'Snowbots Visualizer Mounted')

const router = (
    <Provider store={store}>
        <Router history={history} routes={routes} />
    </Provider>
)

render(router, document.getElementById('app'))

