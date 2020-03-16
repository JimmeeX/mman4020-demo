import React, { useState } from 'react';

import Button from '../components/Button';
import Nav from '../components/Nav';

const Command = () => {
  const [active, setActive] = useState('auto');

  const onClick = () => {
    console.log('Click');
  };

  const buttons =
    active === 'auto'
      ? [
          {
            id: 'button-sample',
            key: 1,
            text: 'Start Sampling',
            large: true,
            color: 'blue'
          },
          {
            id: 'button-purge',
            key: 2,
            text: 'Purge Pump',
            large: true,
            color: 'blue'
          },
          {
            id: 'button-stop',
            key: 3,
            text: 'Emergency Stop',
            large: true,
            color: 'red'
          }
        ]
      : [
          {
            id: 'button-pump',
            key: 1,
            text: 'Turn Pump Off',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-1',
            key: 2,
            text: 'Close Valve 1',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-2',
            key: 3,
            text: 'Close Valve 2',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-3',
            key: 4,
            text: 'Close Valve 3',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-4',
            key: 5,
            text: 'Close Valve 4',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-5',
            key: 6,
            text: 'Close Valve 5',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-6',
            key: 7,
            text: 'Close Valve 6',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-valve-7',
            key: 8,
            text: 'Close Valve 7',
            large: false,
            color: 'blue'
          },
          {
            id: 'button-stop',
            key: 9,
            text: 'Emergency Stop',
            large: true,
            color: 'red'
          }
        ];

  return (
    <div id='command' className='container'>
      <h2>Commands</h2>
      <Nav id='command-nav' active={active} setActive={setActive} />
      <div id={`command-grid-${active}`} className='command-grid'>
        {buttons.map(item => (
          <Button
            {...item}
            style={{
              gridArea: item.id,
              alignSelf: 'center',
              justifySelf: 'center'
            }}
          />
        ))}
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
