# -*- coding: utf-8 -*-
"""
    Copyright (c) 2015 Jonas Böer, jonas.boeer@student.kit.edu

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import warnings
import numpy as np
from numpy.linalg import norm


class MadgwickAHRS:
    samplePeriod = 0.0021
    beta = 0.5
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])

    def __init__(self, quaternion=None, beta=None, samplePeriod=None):
        """
        Initialize the class with the given parameters.
        #:param sampleperiod: The sample period
        :param quaternion: Initial quaternion
        :param beta: Algorithm gain beta
        :return:
        """
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta
        if samplePeriod is not None:
            self.samplePeriod = samplePeriod

    def update(self, gyro, accel, deltaT):
        """
        Perform one update step with data from a IMU sensor array
        :param gyroscope: A three-element array containing the gyroscope data in radians per second.
        :param accelerometer: A three-element array containing the accelerometer data. Can be any unit since a normalized value is used.
        :param deltaT: A float for the update period
        """
        gyro = np.array(gyro)
        accel = np.array(accel)
        q = self.quaternion
        qDot1 = 0.5 * (-q[1] * gyro[0] - q[2] * gyro[1] - q[3] * gyro[2])
        qDot2 = 0.5 * ( q[0] * gyro[0] + q[2] * gyro[2] - q[3] * gyro[1])
        qDot3 = 0.5 * ( q[0] * gyro[1] - q[1] * gyro[2] + q[3] * gyro[0])
        qDot4 = 0.5 * ( q[0] * gyro[2] + q[1] * gyro[1] - q[2] * gyro[0])

        qdot = [qDot1, qDot2, qDot3, qDot4]

        # Normalise accelerometer measurement
        if norm(accel) is 0:
            warnings.warn("accelerometer is zero")
        else:
            accel /= norm(accel)

            #  Auxiliary variables to avoid repeated calculations
            _2q0 = 2.0 * q[0]
            _2q1 = 2.0 * q[1]
            _2q2 = 2.0 * q[2]
            _2q3 = 2.0 * q[3]
            _4q0 = 4.0 * q[0]
            _4q1 = 4.0 * q[1]
            _4q2 = 4.0 * q[2]
            _8q1 = 8.0 * q[1]
            _8q2 = 8.0 * q[2]
            q0q0 = q[0] * q[0]
            q1q1 = q[1] * q[1]
            q2q2 = q[2] * q[2]
            q3q3 = q[3] * q[3]

            # Gradient descent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * accel[0] + _4q0 * q1q1 - _2q1 * accel[1]
            s1 = _4q1 * q3q3 - _2q3 * accel[0] + 4.0 * q0q0 * q[1]- _2q0 * accel[1] - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * accel[2]
            s2 = 4.0 * q0q0 * q[2] + _2q0 * accel[0] + _4q2 * q3q3 - _2q3 * accel[1] - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * accel[2]
            s3 = 4.0 * q1q1 * q[3] - _2q1 * accel[0] + 4.0 * q2q2 * q[3] - _2q2 * accel[1]

            s = np.array([s0, s1, s2, s3])
            s /= norm(s)

            # Apply Feedback Step
            qdot -= self.beta*s #(q * Quaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion /=  norm(q)  # normalise quaternion
