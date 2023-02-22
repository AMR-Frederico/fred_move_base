from time import time


class PositionController: 
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KD = KD 
        self.KI = KI

        self.time = 0
        self.last_time = 0
        self.delta_time = 0

        self.error = 0
        self.last_error = 0
        self.delta_error = 0

        self.integral = 0
    
    def proporcional(self):

        return self.KP * self.error
    
    def integrative(self): 

        self.integral += self.error * self.delta_time
        return self.integral * self.KI

    def derivative(self):
        self.delta_error = self.error - self.last_error

        if(self.delta_error != 0):
            self.delta_error = self.delta_error/self.delta_time
        else:
            self.delta_error = 0
        return self.delta_error*self.KD
            
    def output(self, setPoint, currentValue):

        self.error = setPoint - currentValue

        self.time = time()
        self.delta_time = self.time - self.last_time

        if (self.error != 0):
            output = proporcional() + integrative() + derivative()
        else: 
            output = proporcional() + derivative()
        
        self.last_error = self.error
        self.last_time = self.time

        return output