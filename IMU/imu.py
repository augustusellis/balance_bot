import threading
import BNO055

# Wrapper class for the IMU. This class is meant to combine the BNO055 class
# whatever filter is used, as well as implement threading for faster sensor
# reading.

class imu():

    def __init__(self, pi):

        self.theta = 0 # Value of angle from vertical

        self.bno = BNO055(pi)
        self.bno.begin() # Initialize sensor

        imu_thread = threading.Thread(target = self.run_imu)
        imu_thread.start()


    def run_imu(self):
        # Method runs in separate thread
        while True:
            # Do stuff in loop
            pass
