Program Abstract:

This program calculates Device Trajectory from Gyroscope and Accelerometer data provided in the file imudata.txt. The program runs in the following stages:

Step 1:Calculating device trajectory without sensor error (True Trajectory)
Step 2:Device trajectory after adding simulated sensor errors
Step 3: Creating a GPS Simulation of the device trajectory
Step 4:Using Kalman Filter for GNSS/INS integration to mitigate error
Step 5:Using Kalman Filter on GPS data to mitigate GPS error