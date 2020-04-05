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
  'valve7',
  'flow',
  'depth',
];

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090',
});

const rosTopics = {};

componentNames.map((name) => {
  // Get Topic
  rosTopics[name] = new ROSLIB.Topic({
    ros: ros,
    name: `/arduino/${name}`,
    messageType: ['flow', 'depth'].includes(name)
      ? 'std_msgs/Float32'
      : 'std_msgs/Bool',
  });
  return null;
});

const App = () => {
  const [rosState, setRosState] = useState(false);
  const [depth, setDepth] = useState(null);
  const [flow, setFlow] = useState(null);
  const [pump, setPump] = useState(false);
  const [valve1, setValve1] = useState(false);
  const [valve2, setValve2] = useState(false);
  const [valve3, setValve3] = useState(false);
  const [valve4, setValve4] = useState(false);
  const [valve5, setValve5] = useState(false);
  const [valve6, setValve6] = useState(false);
  const [valve7, setValve7] = useState(false);

  useEffect(() => {
    // Check ROS Connection
    ros.on('connection', () => {
      console.log('Established Connection with ROS');
      setRosState(true);
    });

    ros.on('error', (error) => {
      console.log(`Connection Error: ${error}`);
      setRosState(false);
    });

    ros.on('close', () => {
      console.log('Closed Connection with ROS');
      setRosState(false);
    });

    // Initialise Subscribers
    componentNames.map((name) => {
      rosTopics[name].subscribe((msg) => {
        switch (name) {
          case 'pump':
            setPump(msg.data);
            break;
          case 'valve1':
            setValve1(msg.data);
            break;
          case 'valve2':
            setValve2(msg.data);
            break;
          case 'valve3':
            setValve3(msg.data);
            break;
          case 'valve4':
            setValve4(msg.data);
            break;
          case 'valve5':
            setValve5(msg.data);
            break;
          case 'valve6':
            setValve6(msg.data);
            break;
          case 'valve7':
            setValve7(msg.data);
            break;
          case 'flow':
            setFlow(msg.data);
            break;
          case 'depth':
            setDepth(-1 * msg.data);
            break;
          default:
            break;
        }
      });
      return null;
    });
  }, [rosState]);

  const state = {
    ros: rosState,
    depth,
    flow,
    pump,
    valve1,
    valve2,
    valve3,
    valve4,
    valve5,
    valve6,
    valve7,
  };

  return (
    <div id='app'>
      <Command ros={ros} state={state} />
      <Data state={state} />
      <Diagram />
      <Status state={state} />
    </div>
  );
};

export default App;
