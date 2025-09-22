#!/usr/bin/env python3

# state vector
# position, velocity, orientation (quaternion)
# x = [px, py, pz, vx, vy, vz]  

import numpy as np

class AdaptiveEKF:
    def __init__(self):
        self.dt = 0.05  # 20Hz update rate

        # State vector: [px, py, pz, vx, vy, vz]
        self.x = np.zeros((6, 1))  

        # Covariance matrix
        self.P = np.eye(6) * 0.1

        # Process noise (adaptive)
        self.Q_base = np.eye(6) * 0.01

        # Measurement noise (adaptive)
        self.R_base = np.eye(3) * 0.05

    def predict(self, acc, gyro):
        # Simple motion model: constant velocity + IMU acceleration
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        # Control input: acceleration
        B = np.zeros((6, 3))
        B[3:, :] = np.eye(3) * self.dt

        u = np.array(acc).reshape((3, 1))

        # Adaptive process noise
        acc_norm = np.linalg.norm(acc)
        self.Q = self.Q_base * (1 + acc_norm)

        self.x = F @ self.x + B @ u
        self.P = F @ self.P @ F.T + self.Q

    def update_lidar(self, p_lidar):
        # Measurement matrix: only position
        H = np.zeros((3, 6))
        H[:, :3] = np.eye(3)  # only position measured

        z = np.array(p_lidar).reshape((3, 1))

        # Adaptive measurement noise
        lidar_conf = np.linalg.norm(z)
        self.R = self.R_base * (1 + 1.0 / (lidar_conf + 1e-3))

        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P  
        
    def get_state(self):
        return self.x.flatten()


