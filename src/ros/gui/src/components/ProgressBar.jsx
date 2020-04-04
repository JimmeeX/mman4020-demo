import React from 'react';

const maxWidth = 200;

const ProgressBar = props => {
  const { id, name, current, max, units } = props;

  const currWidth = (current / max) * maxWidth;

  return (
    <div id={id} className='progress-bar'>
      <p className='progress-bar__name'>{name}</p>
      <div className='progress-bar__rect' style={{ width: `${maxWidth}px` }}>
        <div
          className='progress-bar__rect-max'
          style={{ width: `${maxWidth}px` }}
        />
        <div
          className='progress-bar__rect-current'
          style={{ width: `${currWidth}px` }}
        />
      </div>
      <p className='progress-bar__text'>
        {Math.round(current * 100) / 100}/{max} {units}
      </p>
    </div>
  );
};

export default ProgressBar;
