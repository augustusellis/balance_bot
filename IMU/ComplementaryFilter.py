import numpy as np

class ComplementaryFilter():
    def __init__(self, alpha=0.98, beta=0, rollangle=0):
        self.alpha  = alpha
        self.beta = beta
        self.rollangle = rollangle
        self.gyro_only_angle = rollangle
        self.offset = 5.72
    def update(self, zk, dt):
        '''
        zk :: list, [Gx,Gy,Gz,Ax,Ay,Az,Mx,My,Mz]
        '''
        #mag_angle = np.arctan2(zk[7], zk[8])
        accel_angle = np.arctan2(-zk[4], -zk[5])*180/np.pi - self.offset
        gyro_angle = dt*zk[0] + self.rollangle
        self.rollangle = (1-self.alpha - self.beta)*accel_angle + self.alpha*gyro_angle# + self.beta*mag_angle
        #self.gyro_only_angle =  dt*zk[0] + self.gyro_only_angle
