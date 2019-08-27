import { createAction, handleActions } from 'redux-actions'

export default (app) => {
  const reducer = {
    SERVICES_AUTHENTICATION_AUTHENTICATE_PENDING: (state, action) => {
      return ({
        ...state,
        errors: null,
        loading: true,
        valid: false,
        user: null,
        token: null,
        ignorePendingAuth: false
      })
    },

    SERVICES_AUTHENTICATION_AUTHENTICATE_FULFILLED: (state, action) => {
      const user = action.payload.user

      if (state.ignorePendingAuth) {
        // A logout was dispatched between the authentication being started and completed
        app.logout()

        return {
          ...state,
          errors: null,
          loading: false,
          valid: false,
          data: null,
          token: null,
          ignorePendingAuth: false
        }
      }

      return {
        ...state,
        errors: null,
        loading: false,
        valid: true,
        user: user,
        token: action.payloadtoken,
        ignorePendingAuth: false
      }
    },

    SERVICES_AUTHENTICATION_AUTHENTICATE_REJECTED: (state, action) => {
      return {
        ...state,
        // action.payload = { name: "NotFound", message: "No record found for id 'G6HJ45'",
        //   code:404, className: "not-found" }
        errors: action.payload,
        loading: false,
        valid: false,
        data: null,
        token: null,
        ignorePendingAuth: false
      }
    },

    SERVICES_AUTHENTICATION_LOGOUT: (state, action) => {
      app.logout()

      return ({
        ...state,
        errors: null,
        loading: null,
        valid: false,
        user: null,
        token: null,
        // Ignore the result if an authentication has been started
        ignorePendingAuth: state.isLoading
      })
    },

    SERVICES_AUTHENTICATION_USER: (state, action) => {
      let user = state.user
      if (user) {
        user = { ...user, ...action.payload }
      }

      return ({
        ...state,
        errors: null,
        loading: null,
        valid: false,
        user: user,
        // A logout may be dispatched between the authentication being started and completed
        ignorePendingAuth: false
      })
    }
  }

  // ACTION TYPES

  const AUTHENTICATE = 'SERVICES_AUTHENTICATION_AUTHENTICATE'
  const LOGOUT = 'SERVICES_AUTHENTICATION_LOGOUT'
  const USER = 'SERVICES_AUTHENTICATION_USER'

  return {
    // ACTION CREATORS
    // Note: action.payload in reducer will have the value of .data below
    authenticate: createAction(
      AUTHENTICATE, (p) => ({ promise: app.authenticate(p), data: undefined })
    ),
    logout: createAction(LOGOUT),
    user: createAction(USER),

    // REDUCER
    reducer: handleActions(
      reducer,
      {
        errors: null,
        loading: false,
        valid: false,
        user: null,
        ignorePendingAuth: false
      }
    )
  }
}
