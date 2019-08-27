import GA from 'react-ga'
import { services } from '../feathers'
import { UserType } from '../constants'

const snapshot = store => next => action => {
  const result = next(action)

  // Take snapshot after authentication succeeds
  if (action.type === 'SERVICES_AUTHENTICATION_AUTHENTICATE_FULFILLED') {
    return Promise.all([
      services.categories.find(),
      services.foodTags.find(),
      services.users.get('__self')
    ]).then(res => {
      const user = res[2].value

      GA.set({ userId: user.id })
      GA.set({ supplierId: user.supplierId })

      if (user.type === UserType.Admin) {
        return Promise.all([
          services.users.find(),
          services.suppliers.find(),
          services.listings.find(),
          services.clientDevices.find()
        ])
      }

      return Promise.all([
        services.suppliers.get(user.supplierId),
        services.listings.find({ query: { supplierId: user.supplierId } }),
      ])
    }).then(res => {
      console.log('Snapshot complete')
      return result
    })
  }

  return result
}

export default snapshot
