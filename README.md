# Satellite Attitude Control Simulation

<p align="center">

<p align="center">

![GNC](https://img.shields.io/badge/Domain-GNC-red)
![ADCS](https://img.shields.io/badge/System-Attitude_Control-blueviolet)

![MATLAB](https://img.shields.io/badge/MATLAB-Simulation-orange?logo=matlab)

![Method](https://img.shields.io/badge/Control-PD_Control-blue)
![Math](https://img.shields.io/badge/Quaternions-Rotation_Representation-black)
![Physics](https://img.shields.io/badge/Rigid_Body-Dynamics-purple)

![Simulation](https://img.shields.io/badge/Type-Attitude_Dynamics-success)
![Output](https://img.shields.io/badge/Result-3D_Rotation-brightgreen)

![Status](https://img.shields.io/badge/Status-Completed-success)

</p>

</p>

This project demonstrates attitude dynamics and control of a satellite using quaternion-based modeling. The simulation includes rigid-body dynamics, quaternion kinematics, and a PD controller that stabilizes the satellite’s orientation.

---

<p align="center">
  <img src="https://github.com/user-attachments/assets/629fd0e3-9e36-4aa0-9f24-4d2c9507dbee" alt="Satellite Attitude Animation" width="500"/>
</p>

## Quaternion-Based PD Controller for a CubeSat
This project simulates the **attitude dynamics and control** of a CubeSat using quaternion representations. A PD controller is implemented to track a desired satellite orientation, rejecting angular motion and stabilizing the spacecraft.
The simulation is written in GNU Octave (free alternative to MATLAB) and includes both plots of quaternion evolution and 3D animation of the satellite.

---

## ✨ Features
- ✅ Quaternion kinematics (no singularities unlike Euler angles)
- ✅ PD attitude control
- ✅ Plots of quaternion convergence & angular velocity
- ✅ 3D animated CubeSat with live attitude visualization
- ✅ Easily customizable inertia matrix, controller gains, and desired orientation

---

## Theory
### Attitude Dynamics  
The rigid body rotational dynamics are modeled as:

$$
I \dot{\omega} + \omega \times (I \omega) = \tau
$$

where:  
- \( I \) = inertia matrix  
- \( \omega \) = angular velocity  
- \( \tau \) = control torque  

### Quaternion Kinematics
$$
\dot{q} = \frac{1}{2} \, \Omega(\omega) \, q
$$

with  

$$
\Omega(\omega) =
\begin{bmatrix}
0 & -\omega_x & -\omega_y & -\omega_z \\
\omega_x & 0 & \omega_z & -\omega_y \\
\omega_y & -\omega_z & 0 & \omega_x \\
\omega_z & \omega_y & -\omega_x & 0
\end{bmatrix}
$$

### Control Law (PD)
$$
\tau = -K_p \ q_{\text{err,vec}} \-\ K_d \ \omega
$$

where $q_{\text{err}}$ is the quaternion error between current and desired orientation.  

---

## 🚀 Results

### Quaternion Tracking  
<img width="1590" height="425" alt="image" src="https://github.com/user-attachments/assets/236fb0fb-95ed-4e9d-a375-9234c19356a6" />


### Angular Velocity Damping  
<img width="1589" height="428" alt="image" src="https://github.com/user-attachments/assets/b33af8ba-e834-4e5e-bbf1-fa6199aa9daa" />


### 3D CubeSat Animation  
<p align="center">
  <img src="https://github.com/user-attachments/assets/629fd0e3-9e36-4aa0-9f24-4d2c9507dbee" alt="Satellite Attitude Animation" width="500"/>
</p>
