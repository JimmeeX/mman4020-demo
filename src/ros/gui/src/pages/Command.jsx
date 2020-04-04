import React, { Fragment, useState } from 'react';
import ROSLIB from 'roslib';
import { ToastContainer, toast } from 'react-toastify';

import Button from '../components/Button';
import Modal from '../components/Modal';
import Nav from '../components/Nav';

const numJars = 6;
const changeStateTimeout = 10000;

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

// Initialise ActionClients & Action Goals
// Initialise ROS Action Clients

const serverActionNames = [
  { serverName: 'sample', actionName: 'sampler/SampleAction' },
  { serverName: 'purge', actionName: 'sampler/PurgeAction' },
  { serverName: 'stop', actionName: 'sampler/StopAction' },
  { serverName: 'pump', actionName: 'sampler/SetPumpAction' },
  { serverName: 'valve1', actionName: 'sampler/SetValveAction' },
  { serverName: 'valve2', actionName: 'sampler/SetValveAction' },
  { serverName: 'valve3', actionName: 'sampler/SetValveAction' },
  { serverName: 'valve4', actionName: 'sampler/SetValveAction' },
  { serverName: 'valve5', actionName: 'sampler/SetValveAction' },
  { serverName: 'valve6', actionName: 'sampler/SetValveAction' },
  { serverName: 'valve7', actionName: 'sampler/SetValveAction' }
];

const actionClients = {};
serverActionNames.map(({ serverName, actionName }) => {
  actionClients[serverName] = new ROSLIB.ActionClient({
    ros: ros,
    serverName: `/${serverName}`,
    actionName: actionName
  });

  return null;
});

const Command = props => {
  const { state } = props;

  const [active, setActive] = useState('auto');
  const [sampleChoice, setSampleChoice] = useState(Array(numJars).fill(false));
  const [disabled, setDisabled] = useState({
    sample: false,
    purge: false,
    stop: false
  });
  const [sampleETA, setSampleETA] = useState(null);
  const [purgeETA, setPurgeETA] = useState(null);

  const handleSample = () => {
    setDisabled({ ...disabled, sample: true, purge: true });

    const sampleGoal = new ROSLIB.Goal({
      actionClient: actionClients['sample'],
      goalMessage: {
        jars: sampleChoice
      }
    });

    sampleGoal.on('feedback', feedback => {
      setSampleETA(feedback.eta);
    });

    sampleGoal.on('result', result => {
      if (result.success) {
        toast.success(result.message);
      } else {
        toast.error(result.message);
      }
      setDisabled({ ...disabled, sample: false, purge: false });
      setSampleETA(null);
    });

    sampleGoal.send();
    toast.info("Sent 'sample' message to drone.");
  };

  const handlePurge = () => {
    setDisabled({ ...disabled, sample: true, purge: true });
    const purgeGoal = new ROSLIB.Goal({
      actionClient: actionClients['purge'],
      goalMessage: {}
    });

    purgeGoal.on('feedback', feedback => {
      setPurgeETA(feedback.eta);
    });

    purgeGoal.on('result', result => {
      if (result.success) {
        toast.success(result.message);
      } else {
        toast.error(result.message);
      }
      setDisabled({ ...disabled, sample: false, purge: false });
      setPurgeETA(null);
    });

    purgeGoal.send();
    toast.info("Sent 'purge' message to drone.");
  };

  const handleStop = () => {
    setDisabled({ pump: true, purge: true, stop: true });
    const stopGoal = new ROSLIB.Goal({
      actionClient: actionClients['stop'],
      goalMessage: {}
    });

    stopGoal.on('result', result => {
      if (result.success) {
        toast.success(result.message);
      } else {
        toast.error(result.message);
      }
      setDisabled({ pump: false, purge: false, stop: false });
    });

    stopGoal.on('timeout', () => {
      // Send Request to Stop
      toast.error('Stop request timed out');
      stopGoal.cancel();
    });

    stopGoal.send(changeStateTimeout);
    // purgeGoal.cancel();
    // sampleGoal.cancel();
    toast.info("Sent 'stop' message to drone.");
  };

  const handleSetPump = newState => {
    const pumpGoal = new ROSLIB.Goal({
      actionClient: actionClients['pump'],
      goalMessage: {
        state: newState
      }
    });

    pumpGoal.on('result', result => {
      if (result.success) {
        toast.success(result.message);
      } else {
        toast.error(result.message);
      }
    });

    pumpGoal.send(changeStateTimeout);
    toast.info(
      `Sent 'set pump to ${newState ? 'on' : 'off'}' message to drone.`
    );
  };

  const handleSetValve = (id, newState) => {
    const valveGoal = new ROSLIB.Goal({
      actionClient: actionClients[`valve${id}`],
      goalMessage: {
        id: id,
        state: newState
      }
    });

    valveGoal.on('result', result => {
      if (result.success) {
        toast.success(result.message);
      } else {
        toast.error(result.message);
      }
    });

    valveGoal.send(changeStateTimeout);
    toast.info(
      `Sent 'set valve ${id} to ${newState ? 'on' : 'off'}' message to drone.`
    );
  };

  const buttons =
    active === 'auto'
      ? [
          {
            id: 'button-sample',
            key: 1,
            text:
              disabled.sample && sampleETA
                ? `Sampling... (${sampleETA})`
                : 'Start Sampling',
            large: true,
            color: 'blue',
            disabled: disabled.sample,
            popconfirm: {
              title: 'Choose Jars to Sample',
              content: (
                <div className='sample-form'>
                  {sampleChoice.map((choice, i) => (
                    <div key={`checkbox-${i + 1}`}>
                      <div className='checkbox__label'>{`Jar ${i + 1}`}</div>
                      <input
                        type='checkbox'
                        className='checkbox__value'
                        style={{
                          gridArea: `jar${i + 1}`,
                          placeSelf: 'center'
                        }}
                        checked={choice}
                        onChange={() => {
                          const sampleChoiceCopy = sampleChoice.slice(); // Copy of array
                          sampleChoiceCopy[i] = !sampleChoiceCopy[i];
                          setSampleChoice(sampleChoiceCopy);
                        }}
                      />
                    </div>
                  ))}
                </div>
              ),
              onConfirm: () => handleSample()
            }
          },
          {
            id: 'button-purge',
            key: 2,
            text:
              disabled.purge && purgeETA
                ? `Purging... (${purgeETA})`
                : 'Purge Pump',
            large: true,
            color: 'blue',
            disabled: disabled.purge,
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handlePurge()
            }
          },
          {
            id: 'button-stop',
            key: 3,
            text: 'Emergency Stop',
            large: true,
            color: 'red',
            disabled: disabled.stop,
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleStop()
            }
          }
        ]
      : [
          {
            id: 'button-pump',
            key: 1,
            text: `Turn Pump ${state.pump ? 'Off' : 'On'}`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetPump(!state.pump)
            }
          },
          {
            id: 'button-valve-1',
            key: 2,
            text: `${state.valve1 ? 'Close' : 'Open'} Valve 1`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(1, !state.valve1)
            }
          },
          {
            id: 'button-valve-2',
            key: 3,
            text: `${state.valve2 ? 'Close' : 'Open'} Valve 2`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(2, !state.valve2)
            }
          },
          {
            id: 'button-valve-3',
            key: 4,
            text: `${state.valve3 ? 'Close' : 'Open'} Valve 3`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(3, !state.valve3)
            }
          },
          {
            id: 'button-valve-4',
            key: 5,
            text: `${state.valve4 ? 'Close' : 'Open'} Valve 4`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(4, !state.valve4)
            }
          },
          {
            id: 'button-valve-5',
            key: 6,
            text: `${state.valve5 ? 'Close' : 'Open'} Valve 5`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(5, !state.valve5)
            }
          },
          {
            id: 'button-valve-6',
            key: 7,
            text: `${state.valve6 ? 'Close' : 'Open'} Valve 6`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(6, !state.valve6)
            }
          },
          {
            id: 'button-valve-7',
            key: 8,
            text: `${state.valve7 ? 'Close' : 'Open'} Valve 7`,
            large: false,
            color: 'blue',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleSetValve(7, !state.valve7)
            }
          },
          {
            id: 'button-stop',
            key: 9,
            text: 'Emergency Stop',
            large: true,
            color: 'red',
            disabled: disabled.stop,
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleStop()
            }
          }
        ];

  const buttonsList = buttons.map(item => {
    const buttonComponent = (
      <Button
        id={item.id}
        text={item.text}
        large={item.large}
        color={item.color}
        disabled={item.disabled}
        style={{
          gridArea: item.id,
          alignSelf: 'center',
          justifySelf: 'center'
        }}
      />
    );

    return item.popconfirm ? (
      <Modal
        key={item.key}
        {...item.popconfirm}
        disabled={item.disabled}
        style={{
          gridArea: item.id,
          alignSelf: 'center',
          justifySelf: 'center'
        }}
      >
        {buttonComponent}
      </Modal>
    ) : (
      <Fragment key={item.key}>{buttonComponent}</Fragment>
    );
  });

  return (
    <div id='command' className='container'>
      <h2>Commands</h2>
      <Nav id='command-nav' active={active} setActive={setActive} />
      <div id={`command-grid-${active}`} className='command-grid'>
        {buttonsList}
      </div>
      <ToastContainer />
    </div>
  );
};

export default Command;
