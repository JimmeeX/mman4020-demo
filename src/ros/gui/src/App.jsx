import React from 'react';

import Command from './pages/Command';
import Data from './pages/Data';
import Diagram from './pages/Diagram';
import Status from './pages/Status';

const App = () => {
  return (
    <div id='app'>
      <Command />
      <Data />
      <Diagram />
      <Status />
    </div>
  );
};

export default App;
