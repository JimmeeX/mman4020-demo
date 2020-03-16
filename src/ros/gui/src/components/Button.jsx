import React from 'react';
import cx from 'classnames';

const Button = props => {
  const { id, color, large, style } = props;

  const className = cx('button', {
    large: large,
    red: color === 'red',
    blue: color !== 'red'
  });

  return (
    <div id={id} className={className} style={style} onClick={props.onClick}>
      <div className='button-text'>{props.text}</div>
    </div>
  );
};

export default Button;
