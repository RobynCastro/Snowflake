import React, { Component } from 'react';
import NAV2D from './nav2d.js';
import ROS2D from './ros2d.js';
import ROSLIB from 'roslib';
import PropTypes from 'prop-types';

function _classCallCheck(instance, Constructor) { if (!(instance instanceof Constructor)) { throw new TypeError("Cannot call a class as a function"); } }

function _possibleConstructorReturn(self, call) { if (!self) { throw new ReferenceError("this hasn't been initialised - super() hasn't been called"); } return call && (typeof call === "object" || typeof call === "function") ? call : self; }

function _inherits(subClass, superClass) { if (typeof superClass !== "function" && superClass !== null) { throw new TypeError("Super expression must either be null or a function, not " + typeof superClass); } subClass.prototype = Object.create(superClass && superClass.prototype, { constructor: { value: subClass, enumerable: false, writable: true, configurable: true } }); if (superClass) Object.setPrototypeOf ? Object.setPrototypeOf(subClass, superClass) : subClass.__proto__ = superClass; }

export default class Nav2d extends React.Component {
  componentDidMount () {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    })

    let viewer = new ROS2D.Viewer({
      divID: this.props.id,
      width: this.props.width,
      height: this.props.height
    })
    this.nav = NAV2D.OccupancyGridClientNav({
      ros: this.ros,
      rootObject: viewer.scene,
      viewer: viewer,
      serverName: this.props.serverName,
      continuous: true
    })
  }

  componentWillUnmount() {
    this.ros.close()
  }

  render() {
    return React.createElement('div', { id: this.props.id })
  }
}

Nav2d.propTypes = process.env.NODE_ENV !== "production" ? {
  ros: PropTypes.object,
  id: PropTypes.string,
  width: PropTypes.number,
  height: PropTypes.number,
  serverName: PropTypes.string
} : {};
