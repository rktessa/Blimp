# Blimp
A simple autonomous blimp project on Raspberry

## Lavori in corso
In the blimp_class file a repo of all the functions that might be used during blimp flight is under construction.

In file "madgwick.py" it can be found a working
implementation of the madgwick function.

In "kalman_blimp_test_01.py" I am developing the kalman filter for heading estimation of the vehicle during its motion. 

The main program then function in this way: 

![Alt text](https://github.com/rktessa/Blimp/blob/main/alg_flow.jpg?raw=true "Scheme of the  Alghorithm ") 

Important aspects to undestand for Kalman filter implementation:

- orientation of IMU (in particular due to gravity) impact Kalman? 

- calculation of sensor accuracy value for determining the variance of sensors. 

- smooth the value of Kalman filter, see RTS. 
