import { createAction, handleActions } from 'redux-actions'

function createMethodReducer (name, isRemove) {
  const DEFAULT = {
    error: null,
    loading: false,
    result: null,
    store: {},
    keys: []
  }

  return {
    [`${name}_PENDING`]: (state = DEFAULT, action) => ({
      ...state,
      error: null,
      loading: true
    }),
    [`${name}_REJECTED`]: (state = DEFAULT, action) => ({
      ...state,
      error: action.payload,
      loading: false
    }),
    [`${name}_FULFILLED`]: (state = DEFAULT, action) => {
      let result = action.payload
      let ret = {
        ...state,
        error: null,
        loading: false,
        result: result
      }

      if (isRemove) {
        delete ret.store[result.id]
        ret.keys = Object.keys(ret.store)
        return ret
      }

      // Update store
      if (result instanceof Array) {
        for (let i = 0; i < result.length; i++) {
          if (result[i].id == null) {
            console.warn('Received object without valid ID', result[i])
          } else {
            ret.store[result[i].id] = result[i]
          }
        }
      } else {
        if (result.id == null) {
          console.warn('Received object without valid ID', result)
        } else {
          ret.store[result.id] = result
        }
      }

      ret.keys = Object.keys(ret.store)
      return ret
    }
  }
}

function reduxifyService (app, route, name = route) {
  const SERVICE_NAME = `SERVICES_${name.toUpperCase()}_`

  const service = app.service(route)
  if (!service) throw new Error(`Could not find service ${route}`)

  // Action types
  const FIND = `${SERVICE_NAME}FIND`
  const GET = `${SERVICE_NAME}GET`
  const CREATE = `${SERVICE_NAME}CREATE`
  const UPDATE = `${SERVICE_NAME}UPDATE`
  const PATCH = `${SERVICE_NAME}PATCH`
  const REMOVE = `${SERVICE_NAME}REMOVE`
  const RESET = `${SERVICE_NAME}RESET`

  return {
    find: createAction(FIND, (params) => ({ promise: service.find(params) })),
    get: createAction(GET, (id, params) => ({ promise: service.get(id, params) })),
    create: createAction(CREATE, (data, params) => ({ promise: service.create(data, params) })),
    update: createAction(UPDATE, (id, data, params) => ({ promise: service.update(id, data, params) })),
    patch: createAction(PATCH, (id, data, params) => ({ promise: service.patch(id, data, params) })),
    remove: createAction(REMOVE, (id, params) => ({ promise: service.remove(id, params) })),
    reset: createAction(RESET),

    reducer: handleActions(Object.assign({},
      createMethodReducer(FIND),
      createMethodReducer(GET),
      createMethodReducer(CREATE),
      createMethodReducer(UPDATE),
      createMethodReducer(PATCH),
      createMethodReducer(REMOVE, true),
      { [RESET]: (state, action) => {
        if (state.loading) return state
        return {
          ...state,
          error: null,
          loading: false,
          result: null,
          store: {},
          keys: []
        }
      }}
    ), {
      error: null,
      loading: false,
      result: null,
      store: {},
      keys: []
    })
  }
}

export default function (app, routeNameMap) {
  const services = {}

  let names = Object.keys(routeNameMap)
  for (let i = 0; i < names.length; i++) {
    let name = names[i]
    services[name] = reduxifyService(app, routeNameMap[name], name)
  }

  return services
}
