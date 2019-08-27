import feathers from 'feathers/client'
import hooks from 'feathers-hooks'
import rest from 'feathers-rest/client'
import authentication from 'feathers-authentication-client'
import { getServicesStatus, bindWithDispatch } from 'feathers-redux'
import reduxify from './reduxify'
import reduxifyAuth from './reduxify-auth'
import { bindActionCreators } from 'redux'
import { Dev, Staging } from '../constants'

const URL = Dev ? 'http://localhost:9020'
  : Staging ? 'https://api-dev.scrapless.ca'
  : 'https://api.scrapless.ca'
export const API_URL = URL + '/api/'

const restClient = rest(URL)
const client = feathers()

client.configure(hooks())
client.configure(restClient.jquery(window.$))
client.configure(authentication({
  path: '/auth',
  storage: window.localStorage
}))

const SERVICE_NAME_MAP = {
  listingViews: '/api/listing-views',
  listings: '/api/listings',
  suppliers: '/api/suppliers',
  users: '/api/users',
  clientDevices: '/api/client-devices'
}
// TODO: add password-reset endpoints here 'resetPassword': '/reset-password/:action/:value'

export const SINGULAR_SERVICES = ['users', 'suppliers']
export const SERVICE_NAMES = Object.keys(SERVICE_NAME_MAP)
export const SERVICE_PRIORITY = ['auth', ...SERVICE_NAMES]

export const auth = reduxifyAuth(client)
export const services = reduxify(client, SERVICE_NAME_MAP)

export const getStatus = (services, names = SERVICE_PRIORITY) => getServicesStatus(services, names)

export function bindServices (store) {
  let boundServices = bindWithDispatch(store.dispatch, services)
  boundServices.auth = bindActionCreators(auth, store.dispatch)
  return boundServices
}

export default client
