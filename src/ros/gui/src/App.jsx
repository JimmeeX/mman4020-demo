import React, { useState } from 'react';
import ROSLIB from 'roslib';

import Command from './pages/Command';
import Data from './pages/Data';
import Diagram from './pages/Diagram';
import Status from './pages/Status';

const App = () => {
  const [status, setStatus] = useState('Closed');

  const ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090'
  });

  // Check ROS Connection
  ros.on('connection', () => {
    if (status !== 'Connected') {
      console.log('Established Connection with ROS');
      setStatus('Connected');
    }
  });

  ros.on('error', error => {
    if (status !== 'Closed') {
      console.log(`Connection Error: ${error}`);
      setStatus('Closed');
    }
  });

  ros.on('close', () => {
    if (status !== 'Closed') {
      console.log('Closed Connection with ROS');
      setStatus('Closed');
    }
  });

  return (
    <div id='app'>
      <Command ros={ros} />
      <Data ros={ros} />
      <Diagram />
      <Status ros={ros} status={status} />
    </div>
  );
};

export default App;
