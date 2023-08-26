# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys

PACKAGE_PARENT = ".."
SCRIPT_DIR = os.path.dirname(
    os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__)))
)
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params


class Filter:
    """Kalman filter class"""

    def __init__(self):
        pass

    def F(self):
        dt = params.dt

        F_matrix = np.matrix(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ]
        )

        return F_matrix

    def Q(self):
        q = params.q
        dt = params.dt
        q1 = ((dt**3) / 3) * q
        q2 = ((dt**2) / 2) * q
        q3 = dt * q

        Q_matrix = np.matrix(
            [
                [q1, 0, 0, q2, 0, 0],
                [0, q1, 0, 0, q2, 0],
                [0, 0, q1, 0, 0, q2],
                [q2, 0, 0, q3, 0, 0],
                [0, q2, 0, 0, q3, 0],
                [0, 0, q2, 0, 0, q3],
            ]
        )

        return Q_matrix

    def predict(self, track):
        x = self.F() * track.x
        P = self.F() * track.P * self.F().transpose() + self.Q()
        track.set_x(x)
        track.set_P(P)

    def update(self, track, meas):
        H = meas.sensor.get_H(track.x)
        S = self.S(track, meas, H)
        K = track.P * H.transpose() * S.I
        x = track.x + K * self.gamma(track, meas)
        P = (np.identity(params.dim_state) - K * H) * track.P
        track.set_x(x)
        track.set_P(P)
        track.update_attributes(meas)

    def gamma(self, track, meas):
        hx = meas.sensor.get_hx(track.x)
        res_gamma = meas.z - hx
        return res_gamma

    def S(self, track, meas, H):
        cov_s = H * track.P * H.transpose() + meas.R
        return cov_s
