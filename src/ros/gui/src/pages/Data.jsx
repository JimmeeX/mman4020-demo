import React from 'react';

import ProgressBar from '../components/ProgressBar';

const Data = () => {
  const jars = [
    { id: 'jar-1', key: 1, name: '1', current: 45, max: 90, units: 'mL' },
    { id: 'jar-2', key: 2, name: '2', current: 20, max: 90, units: 'mL' },
    { id: 'jar-3', key: 3, name: '3', current: 0, max: 90, units: 'mL' },
    { id: 'jar-4', key: 4, name: '4', current: 90, max: 90, units: 'mL' },
    { id: 'jar-5', key: 5, name: '5', current: 30, max: 90, units: 'mL' },
    { id: 'jar-6', key: 6, name: '6', current: 89, max: 90, units: 'mL' }
  ];

  // const waterLevel = [
  //   {key: 1, }
  // ]

  return (
    <div id='data' className='container'>
      <div id='data-capacities'>
        <h2 id='data-capacities__title'>Jar Capacities</h2>
        <div id='data-capacities__content'>
          {jars.map(item => (
            <ProgressBar {...item} />
          ))}
        </div>
      </div>
      <div id='data-info'>
        <div id='data-info__depth' className='data-info__container'>
          <h2>Water Depth</h2>
          <p>2.27 metres</p>
        </div>
        <div id='data-info__temperature' className='data-info__container'>
          <h2>Temperature</h2>
          <p>15 °C</p>
        </div>
        <div id='data-info__flow' className='data-info__container'>
          <h2>Flow Rate</h2>
          <p>100 mL/min</p>
        </div>
      </div>
    </div>
  );
};

export default Data;
