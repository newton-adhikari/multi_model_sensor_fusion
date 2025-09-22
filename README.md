# 🧠 Multi-Modal Sensor Fusion System

## 🚀 System Overview

This project implements a real-time sensor fusion system using **LiDAR**, **RGB camera**, and **IMU** data to estimate the pose and trajectory of a mobile platform. The fusion is achieved via a manually implemented **Adaptive Extended Kalman Filter (EKF)** in ROS2 using Python and NumPy.

The system synchronizes multi-modal data streams, transforms them into a common reference frame, and fuses them to produce robust state estimates. It also visualizes the fused pose and trajectory in RViz2 and logs data to CSV for offline analysis.

---

## 🎯 Sensor Setup

| Sensor | Topic | Role |
|--------|-------|------|
| LiDAR (`PointCloud2`) | `/scan` | Provides 3D spatial points for position correction |
| RGB Camera (`Image`) | `/camera_sensor/image_raw` | Used for visual context and LiDAR projection |
| IMU (`Imu`) | `/imu/data` | Supplies acceleration and orientation for motion prediction |

### Extrinsic Calibration (Sensor → `base_link`):

- **LiDAR**: 20 cm forward, 10 cm up  
- **Camera**: 10 cm left, 20 cm up  
- **IMU**: 5 cm up  

All sensor data is transformed into the `base_link` frame using homogeneous transformation matrices.

---

## 🧮 EKF Logic and Mathematics

### 🔸 State Vector

The EKF tracks a 6D state vector:

$$
\mathbf{x} = 
\begin{bmatrix}
p_x \\
p_y \\
p_z \\
v_x \\
v_y \\
v_z
\end{bmatrix}
$$

Where:

- \( p_x, p_y, p_z \): Position in 3D space  
- \( v_x, v_y, v_z \): Velocity in 3D space  

---

### 🔸 Prediction Step

Using IMU linear acceleration:

$$
\mathbf{x}_{k|k-1} = F \cdot \mathbf{x}_{k-1} + B \cdot \mathbf{u}
$$

Where:

- \( F \): State transition matrix  
- \( B \): Control input matrix  
- \( \mathbf{u} \): IMU acceleration vector  

---

### 🔸 Update Step

Using LiDAR position measurement:

$$
\begin{aligned}
\mathbf{y} &= \mathbf{z} - H \cdot \mathbf{x}_{k|k-1} \\
\mathbf{K} &= P \cdot H^\top \cdot (H \cdot P \cdot H^\top + R)^{-1} \\
\mathbf{x}_k &= \mathbf{x}_{k|k-1} + \mathbf{K} \cdot \mathbf{y}
\end{aligned}
$$

Where:

- $\\mathbf{z}$: LiDAR measurement  
- $H$: Measurement matrix  
- $P$: State covariance  
- $R$: Measurement noise covariance  
- $\\mathbf{K}$: Kalman gain  
- $\\mathbf{y}$: Innovation (residual)

---

### 🔸 Adaptive Tuning

Noise covariances are dynamically scaled:

- **Process noise**:  
  $Q \propto \|\\mathbf{a}_{\\text{IMU}}\|$

- **Measurement noise**:  
  $R \propto \\frac{1}{\|\\mathbf{z}_{\\text{LiDAR}}\|}$


This improves robustness under varying sensor conditions.

---

## 🖼️ Visualization

Published topics for RViz2:

- `/fused_pose`: Fused position and orientation (`PoseStamped`)  
- `/fused_path`: Accumulated trajectory (`Path`)  
- `/imu_pose`: IMU orientation (`PoseStamped`)  
- `/camera_sensor/image_raw`: RGB image with projected LiDAR points  
- `/scan`: Raw LiDAR point cloud  

---

## 📊 Results

- Real-time fusion at ~20 Hz  
- Accurate pose estimation under moderate motion  
- LiDAR points successfully projected into camera frame  
- Trajectory visualized and logged to `fusion_log.csv`  

---

## ⚠️ Limitations

- No visual odometry or feature tracking from camera yet  
- Only first LiDAR point used for update (can be extended to full cloud)  
- Orientation fusion relies solely on IMU (no magnetometer or visual correction)  
- No outlier rejection or fault detection  

---

## 📁 Files

- `sensor_fusion_node.py`: Main ROS2 node  
- `sensor_subscribers.py`: Script which subscribes to sensor nodes   
- `AdaptiveEFK.py`: EKF implementation  
- `fusion_log.csv`: Logged state estimates  

---

---

## 🚀 Launching the Simulation

After building the workspace, you can launch the full system using:

```bash
ros2 launch multi_model_sensor_fusion multi_model_sensor_fusion.launch.py
```

Before running any standalone scripts, make sure they are executable:
```bash
chmod +x src/multi_model_sensor_fusion/sensor_subscribers.py
chmod +x src/multi_model_sensor_fusion/plot_trajectory.py
```

```bash
./src/multi_model_sensor_fusion/sensor_subscribers.py
./src/multi_model_sensor_fusion/plot_trajectory.py
```

---

## 🧪 Future Work

- Integrate visual odometry or optical flow  
- Fuse full LiDAR cloud with scan matching  
- Add uncertainty visualization (covariance ellipsoids)  
- Publish TF transforms for dynamic visualization  
