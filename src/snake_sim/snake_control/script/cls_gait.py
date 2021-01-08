

import rospy

class gait:
    def __init__(self):
        self.gaitType = str()
        self.ampVertical = float()
        self.ampHorizontal = float()
        self.phaseVertical = float()
        self.phaseHorizontal = float()
        self.timeDelay = float()
        self.timePeriod = float()

    def set_parameters(self, gType='vertical', Amp_V = 0, Amp_H = 0, Phase_V = 0, Phase_H = 0, T_delay = 0, T_period = 0):
        self.gaitType = gType
        self.ampVertical = Amp_V
        self.ampHorizontal = Amp_H
        self.phaseVertical = Phase_V
        self.phaseHorizontal = Phase_H
        self.timeDelay = T_delay
        self.timePeriod = T_period

        return 0 #if no error return 0

    def get_parameters(self):
        return (self.gaitType, self.ampVertical, self.ampHorizontal, self.phaseVertical, self.phaseHorizontal, self.timeDelay, self.timePeriod)

    def clear_parameters(self):
        self.gaitType = ""
        self.ampVertical = 0
        self.ampHorizontal = 0
        self.phaseVertical = 0
        self.phaseHorizontal = 0
        self.timeDelay = 0
        self.timePeriod = 0

        return 0

    
class generator:
    def __init__(self):
        self.gaitType = gait()
        self.lastCall = rospy.Time()
        self.motorTheta = [0] * 16