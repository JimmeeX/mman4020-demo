import React from 'react';

const Diagram = () => {
  return (
    <div id='diagram' className='container'>
      <h1 id='diagram__title'>Water Sampler Control</h1>
      <iframe
        id='diagram__picture'
        title='Sampler'
        src='https://myhub.autodesk360.com/ue2c1e621/shares/public/SH919a0QTf3c32634dcf7641f1f2cb1fe306?mode=embed'
        width='600'
        height='450'
        allowfullscreen='true'
        webkitallowfullscreen='true'
        mozallowfullscreen='true'
        frameborder='0'
      ></iframe>
    </div>
  );
};

export default Diagram;
