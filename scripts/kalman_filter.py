import numpy as np


class ConstantVelocityKalmanFilter:
    def __init__(self, init_noise=0.1, obs_noise_var=5*10e+1, dt=0.1, maha_dist_sq_thresh=10e-7, use_maha=True):
        """
        Kalman Filter with Constant Velocity Model
        https://balzer82.github.io/Kalman/
        """
        # State vector (6x1): [x, y, z, x_dot, y_dot, z_dot]
        self.X = np.matrix([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T

        # State-transition matrix (6x6): x(t) = A * x(t-1) + w(t-1)
        self.A = self._get_state_transition_matrix(dt)

        # Initial uncertainty (6x6)
        self.P = np.eye(6) * init_noise

        # Covariance of the process noise (6x6)
        self.Q = np.eye(6) * 10e-1

        # Observation matrix (3x6) - positions are observed and not velocities
        self.H = np.matrix([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0]])

        # Covariance of the observation noise (3x3)
        self.R = np.eye(3) * obs_noise_var

        # Mahalanobis distance squared threshold
        self.maha_dist_sq_thresh = maha_dist_sq_thresh
        self.use_maha = use_maha
    
    def initialize_state(self, X):
        self.X = X

    @staticmethod
    def _get_state_transition_matrix(dt):
        # State-transition matrix (6x6): x(t) = A * x(t-1) + w(t-1)
        return np.matrix([[1.0, 0, 0, dt, 0, 0],
                          [0, 1.0, 0, 0, dt, 0],
                          [0, 0, 1.0, 0, 0, dt],
                          [0, 0, 0, 1.0, 0, 0],
                          [0, 0, 0, 0, 1.0, 0],
                          [0, 0, 0, 0, 0, 1.0]])

    def predict(self, dt=None):
        if dt is not None:
            self.A = self._get_state_transition_matrix(dt)

        # Predict state
        self.X = self.A * self.X

        # Predict error covariance
        self.P = self.A * self.P * self.A.T + self.Q
        return self.X

    def update(self, Z):
        # Update Kalman gain
        S = np.linalg.inv(self.H * self.P * self.H.T + self.R)
        K = self.P * self.H.T * S

        # Update state estimate using new observation Z
        Y = self.H * self.X

        # Compute Mahalanobis distance
        # if self.use_maha:
        #     err = Z - Y
        #     maha_dist_sq = err.T * S * err
        #     print(maha_dist_sq)
        #     if maha_dist_sq[0] > self.maha_dist_sq_thresh:
        #         return self.X

        self.X = self.X + K * (Z - Y)

        # Update uncertainty covariance - eq. varies due to numerical stability as claimed here:
        # https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
        self.P = (np.eye(6) - K * self.H) * self.P * (np.eye(6) - K * self.H).T + K * self.R * K.T

        # Alternate update equations (mathematically same)
        # self.P = (np.eye(6) - K * self.H) * self.P
        # self.P = self.P - K * (self.H * self.P * self.H.T + self.R) * K.T
        return self.X
