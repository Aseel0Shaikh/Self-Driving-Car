# Perception/Sensor Engineer Interview

[//]: # (Image References)
[image1]: ./p1.png
[image2]: ./p2.png

## Question 1
### Explain a recent project you've worked on.

System Integration.
In this project we used ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following.
![alt text][image1]

### What difficulties did you run into this project that you did not expect, and how did you solve them?

I didn't know how to use ROS to implement that since it was my first time with ROS, but the lessons provided by Udacity and the following system architecture diagram helped overcome that.

## Question 2
### What is the difference between an Extended Kalman Filter and an Unscented Kalman Filter? In what situations would there be larger differences between the two approaches?

The main difference from EKF is that in EKF we take only one point i.e. mean and approximate, while the unscented Kalman filter (UKF) uses a deterministic sampling technique known as the unscented transformation (UT) to pick a minimal set of sample points (called sigma points) around the mean. The sigma points are then propagated through the nonlinear functions, from which a new mean and covariance estimate are then formed.

### In what cases will you prefer an EKF over a UKF?

The unscented Kalman filter has a slightly better performance than the extended Kalman filter when used as a fusion method in a positioning module of an integrated navigation information system.

## Question 3
### What are some of the advantages & disadvantages of cameras, lidar and radar?

#### Camera

Pros:
Cameras can easily distinguish shapes, colours, and quickly identify the type of object based on such information. Hence, it can produce an autonomous driving experience that is very similar to the one produced by a human driver.
Since camera is based on imagery, it is the only sensor with the capability of detecting 2D shapes and colours, making it crucial to reading lanes and pavement markings. With higher resolutions, even fading lines and shapes can be read very accurately. Infrared lighting is also equipped with most modern cameras, making it just as easy to navigate at night.
Camera is relatively cheaper compared to other types of sensors.

Cons:
Poor vision under extreme weather events. Its similarity to the human eye also makes it a major disadvantage under severe weather conditions like snowstorms, sandstorms, or other conditions leading to low visibility. Therefore, the camera is only as good as the human eye. Nevertheless, most people do not expect their car to see better than their eyes and would not fully rely on their car under such extreme conditions. In fact, Tesla had decided to abandon radar and use camera only for its Autopilot system, starting with its newly produced Model 3 and Model Y vehicles. Named Tesla Vision, the system is expected to decrease the frequency of system glitches because of the reduction of confusing signals from radar.

#### Radar

Pros:
The greatest advantage of radar is that the transmission of radio waves is not affected by visibility, lighting, and noise. Therefore, radar performance is consistent across all environmental conditions. It also has been used as the default sensor for emergency braking due to its ability to detect and forecast moving objects coming into the vehicle’s path.

Cons:
The radio waves are highly accurate at detecting objects. Yet, compared to the camera, radar is relatively weak at modeling a perfectly precise shape of the object. As a result, the system might not be able to identify exactly what the object is. For instance, unlike the camera, the radar system normally cannot distinguish bicycles from motorcycles, even though it has no problem determining their speeds.

#### LiDAR

Pros:
Same as radar, LiDAR’s efficacy is not affected by the environmental condition.
LiDAR has a detection range of as far as 100 meters away with a calculation error of less than two centimeters. Hence it is capable of measuring thousands of points at any moment, allowing it to model up a very precise 3D depiction of the surrounding environment.

Cons:
LiDAR requires a significant amount of computing power compared to camera and radar since it calculates hundreds of thousands of points every second and transforms them into actions to provide an accurate 3D model of the environment. It also makes LiDAR prone to system malfunctions and software glitches. Due to the sophistication of the software and the computing resources needed, the price to implement a set of LiDAR sensors is the highest among the three.

### What combination of these (and other sensors) would you use to ensure appropriate and accurate perception of the environment?

To ensure appropriate and accurate perception of the environment I would use a mix of at least two of the three sensors to complement each other and outweigh their weaknesses.

## Question 4
### [Code] Explain the steps behind how an Extended Kalman Filter is implemented.
![alt text][image2]

The filter will recieve the initial measurements from sensors (LIDAR and RADAR) and combined using sensor fusion, then it's put in matrix form. After that it will initialize the new state based on the first measurement.

#### Prediction Step
The prediction step in EKF is the same in KF. The new state is predicted based on the best estimate x (the mean state vector which contains information about the object's position and velocity), we multiply it by the state transition matrix F and add the measurement noise, assuming the object kept going at the same velocity. 

$x'=Fx+v$

$P'=FPF^{T} + Q$ 

P' represents the uncertainty of the object position which may change if the object changed direction, accelerated or decelerated.

#### Update Step
Only the measurement update for the radar sensor will use the extended Kalman filter equations.

$y= z - h(x')$

We update the state by comparing the sensor measurmnts z and the predicted measurments x' based on the uncertinity of each one, h(x') maps values from Cartesian coordinates to polar coordinates.

$S= H_{j}P'H_{j}^{T}+ R$

S is the error function which is calculated based on the uncertainty of the sensor measurement R and the uncertainty of the predicted measurments P'.

$K= P'H_{j}^{T}S^{-1}$

K is the Kalman gain. H is the Jacobian matrix which used to linearize the non-linear function.

$x = x' + K.y$

$P = (I- KH_{j})P$

Then the we update the estimate and the uncirtinity.

