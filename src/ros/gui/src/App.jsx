import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

import Command from './pages/Command';
import Data from './pages/Data';
import Diagram from './pages/Diagram';
import Status from './pages/Status';

const componentNames = [
  'pump',
  'valve1',
  'valve2',
  'valve3',
  'valve4',
  'valve5',
  'valve6',
  'valve7'
];

const rosTopics = {};

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

componentNames.map(name => {
  // Get Topic
  rosTopics[name] = new ROSLIB.Topic({
    ros: ros,
    name: `/arduino/${name}`,
    messageType: 'std_msgs/Bool'
  });
  return null;
});

const App = () => {
  const [state, setState] = useState({
    ros: false,
    baro: false,
    flow: false,
    pump: false,
    valve1: false,
    valve2: false,
    valve3: false,
    valve4: false,
    valve5: false,
    valve6: false,
    valve7: false
  });

  useEffect(() => {
    // Check ROS Connection
    ros.on('connection', () => {
      if (!state.ros) {
        console.log('Established Connection with ROS');
        setState({ ...state, ros: true });
      }
    });

    ros.on('error', error => {
      if (state.ros) {
        console.log(`Connection Error: ${error}`);
        setState({ ...state, ros: false });
      }
    });

    ros.on('close', () => {
      if (state.ros) {
        console.log('Closed Connection with ROS');
        setState({ ...state, ros: false });
      }
    });
  });

  componentNames.map(name => {
    // Subscribe
    rosTopics[name].subscribe(msg => {
      if (state[name] !== msg.data) {
        setState({ ...state, [name]: msg.data });
      }
    });
    return null;
  });

  return (
    <div id='app'>
      <Command ros={ros} state={state} />
      <Data ros={ros} />
      <Diagram />
      <Status state={state} />
    </div>
  );
};

export default App;
