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

const Data = props => {
  const { flow, depth } = props.state;

  const [jar1, setJar1] = useState(0);
  const [jar2, setJar2] = useState(0);
  const [jar3, setJar3] = useState(0);
  const [jar4, setJar4] = useState(0);
  const [jar5, setJar5] = useState(0);
  const [jar6, setJar6] = useState(0);
  const [temp, setTemp] = useState(null);

  // Reason for this Bloated Form is to prevent re-subscription to ROS Topic (since the callback variables stay the same).
  // I can't think of a better solution for the time being
  useEffect(() => {
    dataTopics.map(name => {
      const shortName = name.split('/')[2];
      rosTopics[shortName].subscribe(msg => {
        switch (shortName) {
          case 'jar1':
            setJar1(msg.data);
            break;
          case 'jar2':
            setJar2(msg.data);
            break;
          case 'jar3':
            setJar3(msg.data);
            break;
          case 'jar4':
            setJar4(msg.data);
            break;
          case 'jar5':
            setJar5(msg.data);
            break;
          case 'jar6':
            setJar6(msg.data);
            break;
          case 'temp':
            setTemp(msg.data);
            break;
          default:
            break;
        }
      });
      return null;
    });
  }, []);

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
          <p>{depth ? `${Math.round(depth * 100) / 100} metres` : 'N/A'}</p>
        </div>
        <div id='data-info__temperature' className='data-info__container'>
          <h2>Temperature</h2>
          <p>{temp ? `${Math.round(temp * 100) / 100} °C` : 'N/A'}</p>
        </div>
        <div id='data-info__flow' className='data-info__container'>
          <h2>Flow Rate</h2>
          <p>{flow ? `${Math.round(flow * 100) / 100} mL/sec` : 'N/A'}</p>
        </div>
      </div>
    </div>
  );
};

export default Data;
