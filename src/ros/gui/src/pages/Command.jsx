import React, { Fragment, useState } from 'react';
import ROSLIB from 'roslib';

import Button from '../components/Button';
import Modal from '../components/Modal';
import Nav from '../components/Nav';

const numJars = 6;

const Command = props => {
  const { ros } = props;
  const [active, setActive] = useState('auto');
  const [sampleChoice, setSampleChoice] = useState(Array(numJars).fill(false));

  console.log(sampleChoice);

  // Initialise ROS Action Clients
  const actionClients = {};
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
    { serverName: 'valve6', actionName: 'sampler/SetValveAction' }
  ];

  serverActionNames.map(({ serverName, actionName }) => {
    actionClients[serverName] = new ROSLIB.ActionClient({
      ros: ros,
      serverName: `/${serverName}`,
      actionName: actionName
    });
    return null;
  });

  console.log(actionClients);

  const handleSample = () => {
    console.log('SAMPLE');

    const sampleGoal = new ROSLIB.Goal({
      actionClient: actionClients['sample'],
      goalMessage: {
        jars: sampleChoice
      }
    });

    sampleGoal.on('feedback', feedback => {
      // TODO
      console.log(feedback.capacities);
      console.log(feedback.eta);
    });

    sampleGoal.on('result', result => {
      // TODO
      console.log(result.capacities);
      console.log(result.success);
      console.log(result.message);
    });

    sampleGoal.send();
  };

  const handlePurge = () => {
    console.log('PURGING');

    const purgeGoal = new ROSLIB.Goal({
      actionClient: actionClients['purge'],
      goalMessage: {}
    });

    purgeGoal.on('feedback', feedback => {
      // TODO
      console.log(feedback.eta);
    });

    purgeGoal.on('result', result => {
      // TODO
      console.log(result.success);
      console.log(result.message);
    });

    purgeGoal.send();
  };

  const handleStop = () => {
    console.log('STOPPING');

    const stopGoal = new ROSLIB.Goal({
      actionClient: actionClients['stop'],
      goalMessage: {}
    });

    stopGoal.on('result', result => {
      console.log(result);
    });

    stopGoal.on('timeout', () => {
      console.log('TIMEOUT');
    });

    stopGoal.send(5);
  };

  const buttons =
    active === 'auto'
      ? [
          {
            id: 'button-sample',
            key: 1,
            text: 'Start Sampling',
            large: true,
            color: 'blue',
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
            text: 'Purge Pump',
            large: true,
            color: 'blue',
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
            text: 'Turn Pump Off',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-1',
            key: 2,
            text: 'Close Valve 1',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-2',
            key: 3,
            text: 'Close Valve 2',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-3',
            key: 4,
            text: 'Close Valve 3',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-4',
            key: 5,
            text: 'Close Valve 4',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-5',
            key: 6,
            text: 'Close Valve 5',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-6',
            key: 7,
            text: 'Close Valve 6',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-valve-7',
            key: 8,
            text: 'Close Valve 7',
            large: false,
            color: 'blue',
            popconfirm: null
          },
          {
            id: 'button-stop',
            key: 9,
            text: 'Emergency Stop',
            large: true,
            color: 'red',
            popconfirm: {
              title: 'Do you want to proceed?',
              onConfirm: () => handleStop()
            }
          }
        ];

  const buttonsList = buttons.map(item => {
    const buttonStyle = {
      gridArea: item.id,
      alignSelf: 'center',
      justifySelf: 'center'
    };

    const buttonComponent = (
      <Button
        id={item.id}
        text={item.text}
        large={item.large}
        color={item.color}
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

      {/* {active === 'Auto' ? (
        <div id='command-grid-auto' className='command-grid'>
          {buttons.map(item, i) =>
            <>
              <Button {...item} style={{gridArea: item.id}} />
            </>
          }
        </div>
      ) : (
        <div id='command-grid-manual' className='command-grid'>
          <Button id='button-pump' text='Turn Pump Off' />
          <Button id='button-valve-1' text='Close Valve 1' />
          <Button id='button-valve-2' text='Close Valve 2' />
          <Button id='button-valve-3' text='Close Valve 3' />
          <Button id='button-valve-4' text='Close Valve 4' />
          <Button id='button-valve-5' text='Close Valve 5' />
          <Button id='button-valve-6' text='Close Valve 6' />
          <Button id='button-valve-7' text='Close Valve 7' />
          <Button id='button-stop' large color='red' text='Emergency Stop' />
        </div>
      )} */}
    </div>
  );
};

export default Command;
