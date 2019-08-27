const logger = store => next => action => {
  console.info('>> ' + action.type, action.payload)
  return next(action)
}

export default logger
