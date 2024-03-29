# H-Infinity Controller and Luenberger Observer for Flat Quadruped Robot

This repository provides a framework for implementing an H-Infinity controller and Luenberger observer for a flat quadruped robot using the SRD library.
The SRD library is utilized to generate the robot model and find the implicit constrained linear model.
More information about the SRD library can be found [here](https://github.com/SergeiSa/SRD).

## Matlab Part

The Matlab part of the project involves running several scripts sequentially to generate the robot model.

### Steps to Generate the Robot Model

1. Execute the `step2`, `step3`, and `step4` Matlab scripts in sequence. These scripts are responsible for generating the robot model.
2. Run the main Matlab script to linearize the model, obtain the constraints Jacobian, and calculate its first derivative concerning time.
3. Upon successful execution, you should have five `.mat` files (`An.mat`, `Ar.mat`, `Bn.mat`, `J.mat`, `dJ.mat`) generated.

## Python Part

The Python part uses Linear Matrix Inequalities (LMIs) to solve the H-infinity problem. The MOSEK solver is employed for this purpose.

### Requirements and Execution

1. Ensure that the required Python libraries (such as MOSEK) are installed.
2. Run the Jupyter Notebook `Flat Quadruped H infinity.ipynb`.
3. Input the `.mat` files generated from the Matlab part as inputs to the Python script.
