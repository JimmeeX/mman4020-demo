import React from 'react';

import Command from './pages/Command';
import Data from './pages/Data';
import Diagram from './pages/Diagram';
import Status from './pages/Status';

const App = () => {
  return (
    <div className='App'>
      <h1>Water Sampler Control</h1>
      <Command />
      <Data />
      <Diagram />
      <Status />
    </div>
  );
};

export default App;
