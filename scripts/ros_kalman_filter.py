#!/usr/bin/env python3
import numpy as np
import rospy
from pose_estimation_6d.srv import UpdateKalmanFilterAndPredict
from pose_estimation_6d.srv import InitializeState
from kalman_filter import ConstantVelocityKalmanFilter


class ROSConstantVelocityKalmanFilter(ConstantVelocityKalmanFilter):
    def __init__(self, init_noise, obs_noise_var, dt, maha_dist_sq_thresh, use_maha):
        super().__init__(init_noise, obs_noise_var, dt, maha_dist_sq_thresh, use_maha)

        self.update_kalman_filter_and_predict_srv = rospy.Service("update_kalman_filter_and_predict", UpdateKalmanFilterAndPredict, self.update_and_predict)
        self.initialize_state_srv = rospy.Service("initialize_kalman_state", InitializeState, self.initialize_kalman_state)

    def update_and_predict(self, req):
        kf_update = np.zeros((3, 1))
        kf_update[0] = req.x
        kf_update[1] = req.y
        kf_update[2] = req.z
        x = self.update(kf_update)
        kf_prediction = self.predict(req.dt)
        return [kf_prediction[0], kf_prediction[1], kf_prediction[2]]

    def initialize_kalman_state(self, req):
        x = np.zeros((6, 1))
        x[0] = req.x
        x[1] = req.y
        x[2] = req.z
        self.initialize_state(x)
        return True

if __name__ == '__main__':
    rospy.init_node("constant_velocity_kalman_filter")

    init_noise = rospy.get_param("~init_noise")
    obs_noise_var = rospy.get_param("~obs_noise_var")
    dt = rospy.get_param("~dt")
    maha_dist_sq_thresh = rospy.get_param("~maha_dist_sq_thresh")
    use_maha = rospy.get_param("~use_maha")
    ROSConstantVelocityKalmanFilter(init_noise, obs_noise_var, dt, maha_dist_sq_thresh, use_maha)

    rospy.spin()
