import numpy as np

class ComplementaryFilter():
    def __init__(self, alpha=0.98, beta=0, rollangle=0, angleOffset=0):
        self.alpha  = alpha
        self.beta = beta
        self.rollangle = rollangle - angleOffset
        self.gyro_only_angle = rollangle
        self.offset = angleOffset
    def update(self, zk, dt):
        '''
        zk :: list, [Gx,Gy,Gz,Ax,Ay,Az,Mx,My,Mz]
        '''
        #mag_angle = np.arctan2(zk[7], zk[8])
        self.accel_angle = np.arctan2(-zk[4], -zk[5])*180/np.pi - self.offset
        self.gyro_angle = dt*zk[0] + self.rollangle
        self.rollangle = (1-self.alpha)*self.accel_angle + self.alpha*self.gyro_angle# + self.beta*mag_angle
        #self.gyro_only_angle =  dt*zk[0] + self.gyro_only_angle
