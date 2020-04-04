import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

import ProgressBar from '../components/ProgressBar';

const dataTopics = [
  '/volume/jar1',
  '/volume/jar2',
  '/volume/jar3',
  '/volume/jar4',
  '/volume/jar5',
  '/volume/jar6',
  '/arduino/flow',
  '/arduino/temp',
  '/arduino/depth'
];

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

const rosTopics = {};

dataTopics.map(name => {
  // Get Topic
  const shortName = name.split('/')[2];
  rosTopics[shortName] = new ROSLIB.Topic({
    ros: ros,
    name: name,
    messageType: 'std_msgs/Float32'
  });
  return null;
});

const maxVolume = 90;

const Data = () => {
  // const [state, setState] = useState({
  //   jar1: 0,
  //   jar2: 0,
  //   jar3: 0,
  //   jar4: 0,
  //   jar5: 0,
  //   jar6: 0,
  //   flow: null,
  //   temp: null,
  //   depth: null
  // });

  const [jar1, setJar1] = useState(0);
  const [jar2, setJar2] = useState(0);
  const [jar3, setJar3] = useState(0);
  const [jar4, setJar4] = useState(0);
  const [jar5, setJar5] = useState(0);
  const [jar6, setJar6] = useState(0);
  const [flow, setFlow] = useState(null);
  const [temp, setTemp] = useState(null);
  const [depth, setDepth] = useState(null);

  // Reason for this Bloated Form is to prevent re-subscription to ROS Topic (since the callback variables stay the same).
  // I can't think of a better solution for the time being
  useEffect(() => {
    dataTopics.map(name => {
      const shortName = name.split('/')[2];
      rosTopics[shortName].subscribe(msg => {
        switch (shortName) {
          case 'jar1':
            if (jar1 !== msg.data) setJar1(msg.data);
            break;
          case 'jar2':
            if (jar2 !== msg.data) setJar2(msg.data);
            break;
          case 'jar3':
            if (jar3 !== msg.data) setJar3(msg.data);
            break;
          case 'jar4':
            if (jar4 !== msg.data) setJar4(msg.data);
            break;
          case 'jar5':
            if (jar5 !== msg.data) setJar5(msg.data);
            break;
          case 'jar6':
            if (jar6 !== msg.data) setJar6(msg.data);
            break;
          case 'flow':
            if (flow !== msg.data) setFlow(msg.data);
            break;
          case 'temp':
            if (temp !== msg.data) setTemp(msg.data);
            break;
          case 'depth':
            if (depth !== msg.data) setDepth(msg.data);
            break;
          default:
            break;
        }
      });
    });
  }, []);

  // dataTopics.map(name => {
  //   // Subscribe
  //   const shortName = name.split('/')[2];
  //   rosTopics[shortName].subscribe(msg => {
  //     if (state[shortName] !== msg.data) {
  //       setState({ ...state, [name]: msg.data });
  //     }
  //   });
  //   return null;
  // });

  const jars = [
    {
      id: 'jar-1',
      key: 1,
      name: '1',
      current: jar1,
      max: maxVolume,
      units: 'mL'
    },
    {
      id: 'jar-2',
      key: 2,
      name: '2',
      current: jar2,
      max: maxVolume,
      units: 'mL'
    },
    {
      id: 'jar-3',
      key: 3,
      name: '3',
      current: jar3,
      max: maxVolume,
      units: 'mL'
    },
    {
      id: 'jar-4',
      key: 4,
      name: '4',
      current: jar4,
      max: maxVolume,
      units: 'mL'
    },
    {
      id: 'jar-5',
      key: 5,
      name: '5',
      current: jar5,
      max: maxVolume,
      units: 'mL'
    },
    {
      id: 'jar-6',
      key: 6,
      name: '6',
      current: jar6,
      max: maxVolume,
      units: 'mL'
    }
  ];

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
          <p>{depth ? `${depth} metres` : 'N/A'}</p>
        </div>
        <div id='data-info__temperature' className='data-info__container'>
          <h2>Temperature</h2>
          <p>{temp ? `${temp} °C` : 'N/A'}</p>
        </div>
        <div id='data-info__flow' className='data-info__container'>
          <h2>Flow Rate</h2>
          <p>{flow ? `${flow} mL/min` : 'N/A'}</p>
        </div>
      </div>
    </div>
  );
};

export default Data;
