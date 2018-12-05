import numpy as np

class ComplementaryFilter():
    def __init__(self, alpha=0.98, rollangle=0, angleOffset=0):
        """
        Initializes Complementary Filter.
        Angle = (1-alpha)*accel_angle + (alpha)*gyro_angle
        :param      alpha:          (float) filter weighting
        :param      rollangle:      (float) initial angle offset
        :param      angleOffset:    (float) accelerometer angle raw offset
        """
        self.alpha  = alpha
        self.rollangle = rollangle - angleOffset
        self.gyro_only_angle = rollangle
        self.offset = angleOffset
    def update(self, zk, dt):
        '''
        Update the filter's angle estimate.
        :param zk:  (list) Measurement data [Gx,Gy,Gz,Ax,Ay,Az,Mx,My,Mz]
        :param dt:  (float) time delta between updates
        '''
        self.accel_angle = np.arctan2(-zk[4], -zk[5])*180/np.pi - self.offset
        self.gyro_angle = dt*zk[0] + self.rollangle
        self.rollangle = (1-self.alpha)*self.accel_angle + self.alpha*self.gyro_angle
