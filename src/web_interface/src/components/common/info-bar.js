import React from 'react'

export default class InfoBar extends React.Component {
	render() {
		return(
			<div className='container-fluid info-bar'>
				{this.props.label} ({this.props.units}): {this.props.data} 
	        </div>
		)
	}
}