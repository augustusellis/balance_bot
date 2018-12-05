import time

class PIDController():

    def __init__(self, kp, ki, kd):
        '''
        Initialize the PID controller.
        :param kp:  (float) Proportional gain
        :param ki:  (float) Integral gain
        :param kd:  (float) Differential gain
        '''
        self.error = 0
        self.errorPrevious = 0

        self.eP = 0
        self.eI = 0
        self.eD = 0

        self.kP = kp
        self.kI = ki
        self.kD = kd

        self.u = 0

    def update(self, error, dt=None):
        '''
        Update the PID controller with new error values.
        :param error:   (float) current error
        :param dt:      (float) controller time step
        '''
        # Track Previous Error
        self.errorPrevious = self.error

        # Check for inconsistent error readings (too high spikes)
        if abs(error) < 90:
            self.error = error
        else:
            pass
            #print("#### WARNING error: {}".format(error))

        # Set Errors
        self.eP = self.error
        self.eD = self.error - self.errorPrevious
        self.eI = self.eI + self.error

         # limit integral controller wind-up
        if self.eI*self.kI > 100:
            self.eI = 100/self.kI
        if self.eI*self.kI < -100:
            self.eI = -100/self.kI

        # Determine Controller Output
        self.u = self.eP*self.kP + self.eI*self.kI + self.eD*self.kD

        return self
