Program Abstract:

This program calculates Device Trajectory from Gyroscope data provided in the file DataEx3T1.mat. The program runs on the following principles:

Step 1:Calculating device trajectory without sensor error (True Trajectory)

Step 2:Device trajectory after adding simulated sensor errors

Step 3:Dividing the trajectory area in 12X12 grids and assigning one landmark at
every intersection of the grids.

Step 4:sensing the robot positon based on both the noisy gyro data and 
nearest landmarks.

Step 5:Approximating the true device trajectory using Particle Filter for 
INS/landmark integration to mitigate position error.