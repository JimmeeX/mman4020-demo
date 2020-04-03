import React from 'react';

const Status = props => {
  const { ros, status } = props;

  const state = [
    {
      id: 1,
      name: 'Water Sampler',
      type: 'system',
      connection: status === 'Connected',
      state: status === 'Connected'
    },
    {
      id: 2,
      name: 'Barometric Sensor',
      type: 'sensor',
      connection: false,
      state: false
    },
    {
      id: 3,
      name: 'Flow Rate Sensor',
      type: 'sensor',
      connection: true,
      state: true
    },
    { id: 4, name: 'Pump', type: 'pump', connection: null, state: true },
    { id: 5, name: 'Valve 1', type: 'valve', connection: null, state: true },
    { id: 6, name: 'Valve 2', type: 'valve', connection: null, state: true },
    { id: 7, name: 'Valve 3', type: 'valve', connection: null, state: false },
    { id: 8, name: 'Valve 4', type: 'valve', connection: null, state: true },
    { id: 9, name: 'Valve 5', type: 'valve', connection: null, state: false },
    { id: 10, name: 'Valve 6', type: 'valve', connection: null, state: false },
    { id: 11, name: 'Valve 7', type: 'valve', connection: null, state: false }
  ];

  const connectionToHTML = connection => {
    switch (connection) {
      case true:
        return <p style={{ color: 'green' }}>CONNECTED</p>;
      case false:
        return <p style={{ color: 'red' }}>NOT CONNECTED</p>;
      default:
        return <p>N/A</p>;
    }
  };

  const stateToHTML = (state, type) => {
    switch (type) {
      case 'system':
        return state ? 'ON' : 'OFF';
      case 'sensor':
        return state ? 'ON' : 'OFF';
      case 'pump':
        return state ? 'ON' : 'OFF';
      case 'valve':
        return state ? 'OPEN' : 'CLOSE';
      default:
        return 'N/A';
    }
  };

  return (
    <div id='status' className='container'>
      <h2>Status</h2>
      <table>
        <thead>
          <tr>
            <th>Name</th>
            <th>Connection</th>
            <th>State</th>
          </tr>
        </thead>
        <tbody>
          {state.map(item => (
            <tr key={item.id}>
              <td>{item.name}</td>
              <td>{connectionToHTML(item.connection)}</td>
              <td>{stateToHTML(item.state, item.type)}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
};

export default Status;
