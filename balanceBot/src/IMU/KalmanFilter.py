import numpy as np

class KalmanFilter():
    def __init__(self, x0=np.array([0]), Pk0=np.array([0]), Q=np.array([0.01]), R=np.array([0.01])):
        """
        Initializes Kalman Filter.
        :param      x0:     (float vector)   initial state estimate
        :param      Pk0:    (float 2d array) initial filter gain
        :param      Q:      (float 2d array) process noise
        :param      R:      (float 2d array) sensor uncertainties
        """
        self.xk_hat = x0
        self.Pk = Pk0
        self.Gk = np.array([0])
        self.Q = Q
        self.R = R

        self.A = np.eye(self.xk_hat.shape[0])
        self.B = np.zeros(self.xk_hat.shape[0])
        self.C = np.eye(self.xk_hat.shape[0])


    def update(self, zk, uk):
        '''
        Update the filter's state estimate.
        :param zk:  (list) Measurement data [Gx,Gy,Gz,Ax,Ay,Az,Mx,My,Mz]
        :param uk:  (list) System inputs
        '''
        self.xk_hat = self.A @ self.xk_hat  + self.B @ np.array(uk)
        self.Pk = self.A @ self.Pk @ self.A.T + self.Q

        self.Gk = self.Pk @ self.C.T @ np.linalg.inv(self.C @ self.Pk @ self.C.T + self.R)
        self.xk_hat = self.xk_hat + self.Gk @ (np.array(zk) - self.C @ self.xk_hat)
        Pk = (np.eye((self.Gk @ self.C).shape[0]) - self.Gk @ self.C) @ self.Pk
