import numpy as np

# Stores Current Information about the Water Sampler
HISTORY_SIZE = 5

STD_SAMPLE = 0.2
FLOW_THRESHOLD = 1.0

class Sampler():
    def __init__(self, capacities):

        # Volume of 6 Jars
        self.volume = [0] * len(capacities)
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
        for id in ids:
            if id and not (self.capacities[id] == self.volume[id]): return False
        return True

    def addVolume(self, ids, dt):
        if self.state['pump'] and len(self.flow_rate) > 0 and self.flow_rate[-1] > FLOW_THRESHOLD:
            # Get Total Volume
            volume = dt * max(self.flow_rate[-1], 0)
            print(self.flow_rate)
            print(dt)
            # Distribute Amongst Ids (valves which are full should be ignored & closed)
            for idx, val in enumerate(ids):
                if val:
                    curr_volume = np.random.normal(volume, STD_SAMPLE)
                    print(curr_volume)
                    self.volume[idx] = min(self.capacities[idx], self.volume[idx] + curr_volume)

        # Calculate Eta based on remaining volume && flow_rate
        volume_remaining = 0
        for idx, val in enumerate(ids):
            if val: volume_remaining += (self.capacities[idx] - self.volume[idx])
        print(volume_remaining)

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

