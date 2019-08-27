import React from 'react'
import { Route, IndexRedirect, Redirect } from 'react-router'
import { routerActions } from 'react-router-redux'
import { connectedRouterRedirect } from 'redux-auth-wrapper/history3/redirect'
import locationHelperBuilder from 'redux-auth-wrapper/history3/locationHelper'

import App from './components/app'
import Home from './components/home'

const locationHelper = locationHelperBuilder({})

export default (
  <Route path='/' component={App}>
    <IndexRedirect to='home'/>
    <Redirect path='logout' to='login' />
    <Route path='home' component={Home} />
  </Route>
)
