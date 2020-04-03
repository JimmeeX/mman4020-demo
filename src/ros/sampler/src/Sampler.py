# Stores Current Information about the Water Sampler
HISTORY_SIZE = 5

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

        # Sensor Values (5 most recent)
        self.flow_rate = []
        self.temperature = []
        self.water_depth = []

    def addVolume(self, ids, dt):
        # TODO
        if self.state['pump'] and len(self.flow_rate) > 0:
            # Get Total Volume
            volume = dt * max(self.flow_rate[-1], 0)

            # Distribute Amongst Ids (valves which are full should be ignored & closed)
            # splitVolume = 

    def setState(self, key, value):
        self.state[key] = value

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

