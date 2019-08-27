export const UserType = { Supplier: 0, Director: 1, Admin: 10 }
export const Version = 'v0.2.148-BETA'
export const Dev = !process.env.REACT_APP_STAGING && !process.env.REACT_APP_PROD
export const Staging = process.env.REACT_APP_STAGING
