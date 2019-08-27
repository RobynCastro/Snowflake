import React from 'react'
import ROSLIB from 'roslib'
import Nav2d from '../visualizer'

import VideoFeed from '../common/video-feed'
import InfoBar from '../common/info-bar'

import Gamepad from 'react-gamepad'

export default class Home extends React.Component {
  constructor (props) {
    super(props)
    this.state = {
      speedX: 0.0,
      speedZ: 0.0,

      x: 0,
      y: 0,
      theta: 0,
      linearVelocity: 0,
      angularVelocity: 0,
      scanning: false
    }
  }

  connectHandler (gamepadIndex) {
    console.log(`Gamepad ${gamepadIndex} connected !`)
  }
  disconnectHandler (gamepadIndex) {
    console.log(`Gamepad ${gamepadIndex} disconnected !`)
  }
  buttonChangeHandler (buttonName, down) {
    console.log(buttonName, down)
  }
  axisChangeHandler = (axisName, value, previousValue) => {
    if (axisName === 'RightTrigger') {
      this.setState({
        speedX: value
      })
    } else if (axisName === 'LeftTrigger') {
      this.setState({
        speedX: -value
      })
    } else if (axisName === 'RightStickX') {
      this.setState({
        speedZ: -value
      })
    }
    var twist = this.createTwist(this.state.speedX, this.state.speedZ)
    this.cmdVelPublisher.publish(twist)
  }
  
  buttonDownHandler (buttonName) {
    console.log(buttonName, 'down')
  }
  buttonUpHandler (buttonName) {
    console.log(buttonName, 'up')
  }

  componentDidMount () {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    })

    this.ros.on('connection', function () {
      console.log('Connected to websocket server.')
    })

    this.ros.on('error', function (error) {
      console.log('Error connecting to websocket server: ', error)
    })

    this.ros.on('close', function () {
      console.log('Connection to websocket server closed.')
    })

    this.listener = new ROSLIB.Topic({
      ros: this.ros,
      name: '/turtle1/pose',
      messageType: 'turtlesim/Pose',
      throttle_rate: 100
    })

    // this.listener.subscribe((message) => {
    //   this.setState({
    //     x: this.round(message.pose.pose.position.x, 2),
    //     y: this.round(message.pose.pose.position.y, 2),
    //     theta: this.round(message.pose.pose.orientation.z, 2),
    //     linear_velocity: this.round(message.twist.twist.linear.x, 2),
    //     angular_velocity: this.round(message.twist.twist.angular.z, 2)
    //   })
    // })

    this.listener.subscribe((message) => {
      this.setState({
        x: this.round(message.x, 2),
        y: this.round(message.y, 2),
        theta: this.round(message.theta, 2),
        linearVelocity: this.round(message.linear_velocity, 2),
        angularVelocity: this.round(message.angular_velocity, 2)
      })
    })

    this.cmdVelPublisher = new ROSLIB.Topic({
      ros: this.ros,
      name: '/turtle1/cmd_vel',
      messageType: 'geometry_msgs/Twist',
      throttle_rate: 100
    })
  }

  componentWillUnmount () {
    this.ros.close()
  }

  round (value, decimals) {
    let roundedVal = Number(Math.round(value + 'e' + decimals) + 'e-' + decimals)
    if (isNaN(roundedVal)) { roundedVal = 0 }

    return roundedVal
  }

  createTwist (linX, angZ) {
    return new ROSLIB.Message({
      linear: {
        x: linX,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angZ
      }
    })
  }

  render () {
    return (
      <div className='nav-hsc-container sl-contained-dimmmed'>
        <div className='row'>
          <div className='col-sm-4'>
            <VideoFeed />
          </div>
          <div className='col-sm-8 nav-hsc-occ-grid'>
            <Nav2d
              ros={this.ros}
              id='nav2d'
              width={900}
              height={600}
              serverName='/move_base'
            />
          </div>
        </div>
        <div>
          <InfoBar
            label='Heading'
            data={this.state.theta}
            units='radians'
          />
          <InfoBar
            label='Linear Velocity'
            data={this.state.linearVelocity}
            units='m/s'
          />
          <InfoBar
            label='Angular Velocity'
            data={this.state.angularVelocity}
            units='m/s'
          />
          <Gamepad
            onConnect={this.connectHandler}
            onDisconnect={this.disconnectHandler}
            onButtonChange={this.buttonChangeHandler}
            onAxisChange={this.axisChangeHandler}
          >
            <div />
          </Gamepad>
        </div>
      </div>
    )
  }
}
