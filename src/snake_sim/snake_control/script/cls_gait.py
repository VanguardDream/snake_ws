class gait:
    def __init__(self):
        self.gaitType = str()
        self.ampVertical = float()
        self.ampHorizontal = float()
        self.phaseVertical = float()
        self.phaseHorizontal = float()
        self.timeDelay = float()

    def set_parameters(self, gType='vertical', Amp_V = 0, Amp_H = 0, Phase_V = 0, Phase_H = 0, T_delay = 0):
        self.gaitType = gType
        self.ampVertical = Amp_V
        self.ampHorizontal = Amp_H
        self.phaseVertical = Phase_V
        self.phaseHorizontal = Phase_H
        self.timeDelay = T_delay

        return 0 #if no error return 0

    def get_parameters(self):
        return (self.gaitType, self.ampVertical, self.ampHorizontal, self.phaseVertical, self.phaseHorizontal, self.timeDelay)