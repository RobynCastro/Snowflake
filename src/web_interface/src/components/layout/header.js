import React from 'react'
import Logo from '../app-logo'
import { Link } from 'react-router'

class NavLink extends React.Component {
  render () {
    let active = this.props.location ? this.props.location.pathname === this.props.to : false

    return (
      <li className={(this.props.className + ' ' || '') + 'nav-item' + (active ? ' active' : '')}>
        <Link className='nav-link' to={this.props.to}>{this.props.children}</Link>
      </li>
    )
  }
}

export default class Header extends React.Component {
  onLogoutClick (e) {
    e.preventDefault()

    this.props.services.auth.logout()
  }

  renderNavMenu () {
    return [
      <NavLink key={1} to={'/home'} route={this.props.route}>
        <span className='mdi mdi-home' />
        <span className='nav-link-title'> Home </span>
      </NavLink>,
      <NavLink key={2} to={'/history'} route={this.props.route}>
        <span className='mdi mdi-archive' />
        <span className='nav-link-title'> X-Ray Scan History</span>
      </NavLink>,
      <NavLink key={3} to={'/settings'} route={this.props.route}>
        <span className='mdi mdi-settings' />
        <span className='nav-link-title'> Settings</span>
      </NavLink>,
      <li key={4} className='divider' role='separator' />,
      <li key={5} className='nav-item'>
        <a href='/logout' onClick={e => this.onLogoutClick(e)}>
          <span className='mdi mdi-logout' />
          <span className='nav-link-title'> Logout</span>
        </a>
      </li>
    ]
  }

  render () {
    let navMenu = (
      <div className='collapse navbar-collapse' id='sl-main-nav' aria-expanded='false'>
        <ul className='hidden-xs nav navbar-nav navbar-right'>
          <li>
            <a
              className='user dropdown-toggle'
              data-toggle='dropdown'
              role='button'
              aria-haspopup='true'
              aria-expanded='false'>
              <span className='mdi mdi-account-outline' />
              <span className='mdi mdi-menu-down' />
            </a>
            <ul className='dropdown-menu'>
              {this.renderNavMenu()}
            </ul>
          </li>
        </ul>

        <ul className='visible-xs-block nav navbar-nav nav-mobile'>
          <li className=' nav-item disabled'>
            <a>
              <span className='mdi mdi-account-outline' />
            </a>
          </li>
          {this.renderNavMenu()}
        </ul>
      </div>
    )
    return (
      <nav className='navbar navbar-default navbar-static-top sl-navbar'>
        <div className={'container'}>

          {navMenu}

          <div className='navbar-header'>
            <button
              className='navbar-toggle collapsed'
              type='button'
              data-toggle='collapse'
              data-target='#sl-main-nav'
              aria-expanded='false'>
              <span className='sr-only'>Toggle navigation</span>
              <span className='icon-bar' />
              <span className='icon-bar' />
              <span className='icon-bar' />
            </button>

            <Link to='/' className='navbar-brand'>
              <div className='sl-logo'>
                <Logo />
              </div>
              <span>
                <div className='header-text'>
                  Snowbots Web Interface
                </div>
              </span>
            </Link>
          </div>

        </div>
      </nav>
    )
  }
}
