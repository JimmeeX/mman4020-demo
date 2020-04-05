import numpy as np

# Stores Current Information about the Water Sampler
HISTORY_SIZE = 5

STD_SAMPLE = 0.2
FLOW_THRESHOLD = 1.0
DEPTH_THRESHOLD = 0.0

class Sampler():
    def __init__(self, capacities):

        # Volume of 6 Jars
        self.volume = [0.0] * len(capacities)
        self.capacities = capacities # Max Volume each jar can hold

        # State of Components & Sensors
        self.state = {
            'baro': False,
            'flow': False,
            'pump': False,
            'valve1': False,
            'valve2': False,
            'valve3': False,
            'valve4': False,
            'valve5': False,
            'valve6': False,
            'valve7': False,
        }

        # Recommend to Purge before sampling
        self.is_purged = False

        # Sensor Values (5 most recent)
        self.flow_rate = []
        self.temperature = []
        self.water_depth = []

    def isFull(self, ids):
        return self.numJarsLeft(ids) == 0

    def numJarsLeft(self, ids):
        count = 0
        for idx, val in enumerate(ids):
            if val and not np.isclose(self.capacities[idx], self.volume[idx], rtol=1e-05, atol=1e-08, equal_nan=False):
                count += 1
        return count

    def addVolume(self, ids, dt):
        num_jars = self.numJarsLeft(ids)
        """
        Requirements:
        - Pump is Running
        - A measurable flow rate
        - The tube is underwater
        """
        if self.state['pump'] and len(self.flow_rate) > 0 and self.flow_rate[-1] > FLOW_THRESHOLD and len(self.water_depth) > 0 and self.water_depth[-1] < DEPTH_THRESHOLD:
            # Get Total Volume
            volume = dt * max(self.flow_rate[-1], 0)
            # Distribute Amongst Ids (valves which are full should be ignored & closed)
            for idx, val in enumerate(ids):
                if val:
                    curr_volume = np.random.normal(volume / float(num_jars), STD_SAMPLE)
                    self.volume[idx] = min(self.capacities[idx], self.volume[idx] + curr_volume)

        # Calculate Eta based on remaining volume && flow_rate
        volume_remaining = 0
        for idx, val in enumerate(ids):
            if val: volume_remaining += (self.capacities[idx] - self.volume[idx])

        # Calculate Eta
        eta = volume_remaining // max(self.flow_rate[-1],1) # Prevent Division by 0
        return eta


    def setState(self, key, value):
        self.state[key] = value


    def setPurged(self, value):
        self.is_purged = value


    def processFlow(self, value):
        self.flow_rate.append(value)
        if len(self.flow_rate) > HISTORY_SIZE:
            self.flow_rate.pop(0)


    def processTemp(self, value):
        self.temperature.append(value)
        if len(self.temperature) > HISTORY_SIZE:
            self.temperature.pop(0)


    def processDepth(self, value):
        self.water_depth.append(value)
        if len(self.water_depth) > HISTORY_SIZE:
            self.water_depth.pop(0)

