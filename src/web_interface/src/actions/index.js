export default {
  modalShow: (modal, props) => ({
    type: 'MODAL_SHOW',
    payload: { modal, props }
  }),
  modalHide: (modal) => ({
    type: 'MODAL_HIDE',
    payload: { modal }
  })
}
