import threading, time
from collections import deque
from IMU.BNO055 import BNO055
from IMU.ComplementaryFilter import ComplementaryFilter

# Wrapper class for the IMU. This class is meant to combine the BNO055 class
# whatever filter is used, as well as implement threading for faster sensor
# reading.

class IMU():

    def __init__(self, pi, rst):

        self.theta_eul = 0 # Value of angle from vertical from sensor's calculated Euler angles
        self.theta_filt = 0 # Filtered value of theta

        self.bno = BNO055(pi=pi,rst=rst)

        self.cfilt = ComplementaryFilter(alpha=0.98, rollangle=0)

        self.kill = 0


    def start_imu(self):
        self.imu_thread = threading.Thread(target = self.run_imu)
        self.imu_thread.start()


    def run_imu(self):
        # Method runs in separate thread
        print("Starting IMU Thread.")
        current_time = time.time()
        while not self.kill:
            # Do Timing
            previous_time = current_time
            current_time = time.time()

            # Get data from IMU
            gx, gy, gz = self.bno.read_gyroscope()
            ax, ay, az = self.bno.read_accelerometer()
            heading, roll, pitch = self.bno.read_euler()
            self.theta_eul = pitch # Control should try to keep pitch at 0

            # Update Data Filter
            self.cfilt.update([gx,gy,gz,ax,ay,az], current_time-previous_time)

            self.theta_filt = self.cfilt.rollangle

            #print('IMU dT: {}'.format(current_time-previous_time))

        print('Ending IMU.')


    def stop_imu(self):
        self.kill = 1
        self.imu_thread.join()
