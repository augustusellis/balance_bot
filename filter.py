import numpy as np
import pandas
import matplotlib.pyplot as plt
from madgwickahrs import MadgwickAHRS
from KalmanFilter import KalmanFilter

Q = 0.1
R = 250*0.05

kalman = KalmanFilter(np.array([[0]]), np.array([[1]]), np.array([[Q]]), np.array([[R]]))

time = np.linspace(0, 40, 10000)

wx_clean = 50*np.sin(time)

noise_gauss = np.random.normal(0, R, wx_clean.shape)
wx_noise = wx_clean + noise_gauss

wx_kalman = np.zeros(wx_noise.shape)
for i in range(time.shape[0]):
    wx_kalman[i] = kalman.xk_hat[0][0]
    #print(kalman.xk_hat[0][0])
    kalman.update(np.array(wx_noise[i]), np.array([0]))



plt.rcParams.update({'font.size': 22})
plt.plot(time, wx_noise, 'b*', label = 'Noise Added', alpha=0.2)
plt.plot(time, wx_kalman, 'm', label = 'Kalman Filtered', linewidth=7.0)
plt.plot(time, wx_clean, 'c', label = 'Clean Signal', linewidth=7.0)
plt.xlabel('Time, [s]')
plt.ylabel('Wx, [deg/s]')
plt.legend()
plt.grid()
plt.show()

'''
print(np.array(0))
print(np.shape(np.array(0)))
print('----')
print(np.array([0]))
print(np.shape(np.array([0])))
'''
