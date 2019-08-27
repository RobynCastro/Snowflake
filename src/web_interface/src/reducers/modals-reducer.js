const DEFAULT = {
  adminCreate: { open: false }
}

export default function (state = DEFAULT, action) {
  switch (action.type) {
    case 'MODAL_SHOW':
      return {
        ...state,
        [action.payload.modal]: {
          open: true,
          props: action.payload.props
        }
      }

    case 'MODAL_HIDE':
      return {
        ...state,
        [action.payload.modal]: {
          ...state[action.payload.modal],
          open: false
        }
      }

    default:
      return state
  }
}
