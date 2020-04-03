import React, { useState } from 'react';

const Modal = props => {
  const { children, title, content, onConfirm, style } = props;
  const [visible, setVisible] = useState(false);

  return (
    <div className='modal' style={style}>
      {visible && (
        <div className='modal__popup'>
          <div className='modal__body'>
            <h3 className='modal__body__title'>{title}</h3>
            <div className='modal__body__content'>{content}</div>
            <button
              className='modal__body__cancel'
              onClick={() => setVisible(false)}
            >
              Cancel
            </button>
            <button
              className='modal__body__ok'
              onClick={() => {
                setVisible(false);
                onConfirm();
              }}
            >
              OK
            </button>
          </div>
        </div>
      )}
      <div className='modal__main' onClick={() => setVisible(true)}>
        {children}
      </div>
    </div>
  );
};

export default Modal;
