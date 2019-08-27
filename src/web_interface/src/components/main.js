import React from 'react'

import Header from './layout/header'

const FULLPAGES = ['/login', '/login-admin']

export default class Main extends React.Component {
  render () {
    const fullpage = FULLPAGES.indexOf(this.props.location.pathname) >= 0

    return (
      <div style={{ height: '100%' }}>
        { !fullpage ? <Header {...this.props} /> : null }
        { !fullpage ? <div className='navbar-spacer' /> : null }

        {React.Children.map(this.props.children, (child) => React.cloneElement(child, this.props))}
      </div>
    )
  }
}
