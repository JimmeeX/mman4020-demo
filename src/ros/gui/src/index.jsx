import React from 'react';
import ReactDOM from 'react-dom';
import App from './App';
import * as serviceWorker from './serviceWorker';

import ROSLIB from 'roslib';

var ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected');
  // document.getElementById('status').innerHTML = 'Connected';
});

ros.on('error', function(error) {
  console.log('Error');
  // document.getElementById('status').innerHTML = 'Error';
});

ros.on('close', function() {
  console.log('Closed');
  // document.getElementById('status').innerHTML = 'Closed';
});

var txt_listener = new ROSLIB.Topic({
  ros: ros,
  name: '/txt_msg',
  messageType: 'std_msgs/String'
});

txt_listener.subscribe(function(m) {
  document.getElementById('msg').innerHTML = m.data;
});

ReactDOM.render(<App />, document.getElementById('root'));

// If you want your app to work offline and load faster, you can change
// unregister() to register() below. Note this comes with some pitfalls.
// Learn more about service workers: https://bit.ly/CRA-PWA
serviceWorker.unregister();
