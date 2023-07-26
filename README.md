# RBE501Final
Our code is run simply by putting in the desired starting and ending configurations of the robot into `jointInitialPos_Vel`, `jointTargetPos`, and `jointTargetVel`.

`jointInitialPos_Vel` is a 12x1 matrix in which the first 6 elements represent the starting angle of each joint (in radians) and the next 6 elements represent the starting velocity (in m/s) of each joint.
IMPORTANT NOTE: The home configuration of the ABB IRB 1600 is a singularity and likely to cause errors. Therefore, it is not recommended to start the robot at [0,0,0,0,0,0]. Clearing the 5th joint will easily remove the singularity configuration.

`jointTargetPos` is a 6x1 matrix which represents the final angle of each joint.
`jointTargetVel` is a 6x1 matrix which represents the final velocity of each joint.

Once these three parameters are set, you can simply hit the "Run" button in the MATLAB editor tab and the simulation will run to completion.
NOTE: It will take about 30-45 seconds for the simulation to begin based on your hardware.
If the simulation is taking too long to run or if the robot is not reaching the desired point in time. You can also change the `Tf` variable to increase or decrease the simulation time. 

Additional Properties:
There are two variables called `via_pt1` and `via_pt2` respectively. 
Each of these variables represents a point that the user can define as a point for the robot to go to before it starts its trajectory towards the final point.
Each one is a 12x1 matrix representing the desired position and velocity. Additionally, in order to use via points you must change the variables `j` and `k` in the animation for loop to be 1:4 instead of simply 1.

There is a `set_box_mass` function which allows the user to change the mass of the box that the robot carries as well. 
This function is called by typing into the MATLAB terminal: `set_box_mass(robot, 'desired mass (in kg)')`
IMPORTANT NOTE: The real life ABB IRB 1600 has a maximum load of 10kg. Since the urdf is based off the mass and inertial properties of the real robot, anything above 10kg is likely to cause issues.

