import React from 'react'

export default class Footer extends React.Component {
  render () {
    return (
      <div className='sl-footer'>
        <div className='container'>
          <div className='text-center footer-links'>
            <a href='//www.scrapless.ca/support'>Support</a>
            <span>&#x25C6;</span>
            <a href='//www.scrapless.ca/privacy'>Privacy</a>
            <span>&#x25C6;</span>
            <a href='//www.scrapless.ca/terms'>Terms of Service</a>
          </div>
        </div>
      </div>
    )
  }
}
