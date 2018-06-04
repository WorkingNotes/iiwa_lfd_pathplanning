# Introduction
This repository contains code and information for the computer exercises for the tutorial on Learning from Demonstration at ICRA 2016.

These exercises cover mainly representation and learning at the trajectory level, and should not be seen as a comprehensive set of exercises about the huge field of LfD.

These exercises use a simple two-link robot with dynamics from [Peter Corkes's robotics toolbox](http://petercorke.com/Robotics_Toolbox.html)[1]. 

To make sure everything runs smoothly, we recommend disabling your regular matlab path for these exercises. You can set everything up automatically by simply running the `setup_lfd_tutorial.m` script. 

# Exercise 1
The promise of Learning from Demonstration is to reduce the task of programming the robot to simply showing it what it should do. Perhaps the simplest possible way to achieve something like that is to simply record a trajectory and have the robot try to "play it back" exactly.

We encourage you to adjust any parameters you find, and rerun the function to understand the effect on the behavior of the robot. 

### Step 1
There are numerous ways in which we can give demonstrations to a robot. Since we will be working with planar task-spaces in these exercises, we can simply draw trajectories on the robot's task space. First, open the file `exercise_1.m` and run it in matlab. A figure with the two-link robot as well as a dashed line delimiting its workspace is drawn. You are asked to provide a demonstration of a trajectory.

Keep in mind that you are providing a demonstration in *the workspace of the robot*, and hence if any part of the demonstration lies outside the boundary the robot will not be able to play it back. 

### Step 2
Your demonstration consists of a series of x,y coordinates unevenly distributed in time. In order to execute the trajectory with the robot, we need access to at least a velocity profile and preferably also acceleration profile of the trajectory. The get a continuous representation that provides all these data, we can fit a spline representation to our demonstration.

With a spline representation, we don't need to use all of our demonstrated points. We can reduce the data set to a set of knots, which will be used to fit the spline. You can experiment with different number of knots by adjusting the nb_knots parameter in the matlab code. 

```matlab
nb_knots = 10;
disp(sprintf('you now see a spline with %d knots in the figure.', nb_knots));
% fit a spline to the demonstration
nb_data = size(data,2);
skip = floor(nb_data/nb_knots);
knots = data(:,1:skip:end);
```
Cubic splines is one possibility of interpolating the trajectory but there are also many others, feel free to test other methods built in to matlab by changing `'spline'` the following line of code:

```matlab
ppx = interp1(knots(3,:)',knots(1:2,:)','spline','pp');
```
Once we have generated the reduced data set we can interpolate it. The result is a series of 3rd degree polynomials which can be easily differentiated to yield the velocity and acceleration profile. 

```matlab
% and get first and second derivative
ppxd = differentiate_spline(ppx);
ppxdd = differentiate_spline(ppxd);
```

### Step 3
If you have given a demonstration and see the figure with the spline representation of your demonstration, you can press enter to start a simulation. The simulation uses a rather simple Cartesian controller with imperfect dynamic compensation, so it the trajectory may not be followed exactly if you gave a complex/fast demonstration.

### Step 4
Now suppose that the robot finds itself in another starting point, and we would like it to execute the task. The problem here is that we have not really demonstrated or encoded a task, but just a time-dependent trajectory. A naive approach of trying to use this trajectory with other starting points will yield bad performance and is potentially unstable and very dangerous behavior. Try it out by clicking at a new location to start a simulation. The program will repeatedly ask you to click at new locations to simulate from that starting point.

At any time, you can abort the program by either rerunning the file or pressing `ctrl+c` in the matlab console.


# Excercise 2
In the previous exercise, we saw an example of a very basic record and replay approach which as we have seen is prone to error if the slightest thing in the environment changes between demonstration and execution.

A large body of work in LfD has been devoted to finding alternative means of representing demonstrated trajectories, so that the robot can *generalize* the demonstrations to new situations (e.g. new starting points). Many of these methods have been mentioned in the tutorial session. Here, we will focus on Dynamical Systems (DS) representations of a task. Specifically, we will use time-invariant DS representations. The methods used in this exercise is Stable Estimator of Dynamical Systems, SEDS [2].

### Step 1
Open the file `exercise_2.m` and run it. Again, you will see the two-link robot appearing in a figure with a workspace limit shown by a dashed line. Start by again giving a demonstration of a path in the robot workspace.

### Step 2
The source code starts with collection of a demonstration and processing of the demonstration to find velocity etc using splines, just as in Exercise 1. Scroll down until you find the following code:

```matlab
[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,nb_gaussians); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solver
```
In the first of line of the code above creates a Gaussian Mixture Model(GMR) representing the joint probability density function of the demonstrated position and velocity. By conditioning this distribution, so that we get a distribution of velocity *given* position, we are getting close to something we can use for control. However, if we just used the first estimate, we would have an *unstable* model, with diverging trajectories.

The second line of code starts the SEDS optimizer, which modifies the GMM such that it yields a stable DS when conditioned. This is done in a constrained optimization procedure. Depending on the complexity of your demonstration, it is possible that no stable solution can be found, or a solution that does not fit the data well. Rerun the program until the optimization finishes successfully. 

### Step 3
Now that we have a DS model, inspect the behavior by launching simulations from different parts of the workspace by clicking there. As you can see, the robot always follows smooth trajectories that end up at the demonstrated end point.

### Step 4
So far, we have only given one demonstration. In order to generalize, it is generally good to provide several demonstrations. Find the parameter `nb_demo` near the top of the main function and change a higher number, e.g. 2 or 3. You can now give several demonstrations. The demonstrations will be translated so that they all end at the same point. Try demonstrating different trajectories in different parts of the workspace. 

### Step 5
So far, we have used a GMM with a single Gaussian to model the DS. To model more complex data, more Gaussians can be required. Find the parameter `nb_gaussians` and change it to a higher number, e.g. 2.

Note that it is generally a good idea to keep the number of Gaussians low, as the optimization is more likely to end up in a bad local minima with more Gaussians. It is not always possible to get a very accurate reproduction of the trajectory this way. SEDS should not be seen as a precision tool but rather a method that can capture the general motion pattern and generalize it.

### Step 6
Simulate a couple of trajectories by clicking at different starting points. Rerun the program and adjust the parameters to see how it influences the result.

The cost function used in the optimization is very important. So far, you have used the maximum likelihood cost-function, which makes the optimization try to fit the Gaussians so that the resulting GMM represents the demonstrated data with high probability. An alternative cost-function is MSE, which does not care about probabilistic representation of the data but rather tries to minimize the error between observed velocity and model generated velocity (via GMR) at the same locations in the position space. You can change to the MSE cost-function by changing the following line of code near the top of the file:

```matlab
options.objective = 'mse';    % 'likelihood'
```


# Exercise 3
In Exercise 2 we introduced time-invariant DS as a way of encoding motions, and we explored the SEDS algorithm for learning a DS on a particular form (GMR) from demonstrations.

SEDS is a global method, in the sense that it tries to generalize the demonstrations to any part of the state space. As you have seen in the previous exercise, the result is not always suitable and the cost of generalization is local precision. The demonstrated trajectories were not reproduced to the same level of accuracy as in the time-dependent record-and-replay exercise. Furthermore, increasing model complexity by increasing the number of Gaussians often yields unreliable models. 

In this exercise, we will explore *local* learning in DS representations. We will use SEDS as a global baseline model, and then we will *reshape* it locally in order to improve accuracy near the demonstrations. We will use the method described in [3].

### Step 1
Open the file `exercise_3.m` and run it. Again, you will see the two-link robot appearing in a figure with a workspace limit shown by a dashed line. Start by giving a demonstration of a path in the robot workspace. First, we have to learn the Baseline SEDS model that will later be refined locally around the demonstration. If you inspect the code in matlab you will recognize most of it from exercise 2. 

### Step 2
Give demonstration(s) just like in exercise 2. The SEDS optimizer will run and display the resulting SEDS model, which is global but with limited accuracy. Press enter in order to apply local reshaping. You will see a red shade appearing near the demonstrations, and the streamlines aligning much better with the demonstrations than before. 

In order to reshape the DS, there are three things happening:
#### Compute reshaping data set
After the SEDS model has been learned, it can be used to predict velocities at the same locations as where we got demonstrations. Hence, for each position where we have a demonstrated velocity, we can compare it with the velocity according to SEDS at that same point. In the 2d case, we can represent the difference by a rotation angle and a speed scaling between these two velocities. This is done in the following block of

```matlab
lmds_data = [];
for i =1:nb_demo
    dsi = 1+(i-1)*nb_clean_data_per_demo; % demonstration start index
    dei = i*nb_clean_data_per_demo; % demonstration end index
    lmds_data = [lmds_data, generate_lmds_data_2d(Data(1:2,dsi:dei),Data(3:4,dsi:dei),ds(Data(1:2,dsi:dei)),0.05)];
end
```
You can open the function generate_lmds_data_2d to see that really all that is done is computing an angle and a speed scaling.

#### Model reshaping parameters with Gaussian Processes
From the procedure above, we have a data set of positions and corresponding rotation angles and speed scalingss. The rotation angles and speed scalings are called the reshaping parameters. We want to be able to predict reshaping parameters for any position in the workspace. Therefore, we use a Gaussian Process (GP) to model the reshaping parameters over the workspace. We use the gpml toolbox for matlab [4].

Near the top of the file you will find these parameters, which determine the behavior of the GP covariance function:

```matlab
% hyper-parameters for gaussian process
% these can be learned from data but we will use predetermined values here
ell = 0.1; % lengthscale. bigger lengthscale => smoother, less precise ds
sf = 1; % signal variance 
sn = 0.4; % measurement noise
```

#### Reshape the dynamics
Now we have generated a data set and defined a Gaussian Process to define it. The last step is to compute the reshaped DS for any position input. This is done by first computing the SEDS output for the query point, then predicting the reshaping paramters at the query point and lastly applying the reshaping (rotation and scaling) to the SEDS output. The following inline function combines all this:

```matlab
% we now define our reshaped dynamics
% go and have a look at gp_mds_2d to see what it does
reshaped_ds = @(x) gp_mds_2d(ds, gp_handle, lmds_data, x);
```

### Step 3
Now that we have a DS model, inspect the behavior by launching simulations from different parts of the workspace by clicking there. As you can see, the robot always follows smooth trajectories that end up at the demonstrated end point.

### Step 4
As in exercise 2, change the number of demonstrations to see how different behaviors can be learned in different parts of the state-space. You can also experiment to see what happens if conflicting demonstrations are given in the same region. 

### Step 5
The hyper-parameters for GPR with gaussian covariance function are the measurement noise `s_n`, the signal variance `s_f` and the kernel width (also called lengthscale) `ell`. You can find these parameters near the top of the file.

Try to doubling or tripling the kernel width. Then try setting it to a smaller value. As you can see, the effect of the lengthscale is very important, and essentially boils down to a trade-off between generalization and accuracy. 

# Exercise 4
In this exercise we will design a simple variable compliance control law for the robot that takes into account model uncertainty. We will used a similar method to the one described in [2].

### Step 1
Open the file `exercise_4.m` and run it. Again, you will see the two-link robot appearing in a figure with a workspace limit shown by a dashed line. Start by giving a demonstration of a path in the robot workspace. First, we will learn a LMDS model. Again, if you inspect the code in matlab you will recognize most of it from exercise 3. 

### Step 2
Once we have a model we can design our controller. If we now have a look at the simulation_opt_control function, it is very similar to the previous exercise simulation function but it has some additional lines

```matlab
% Get open loop trajectory from the DS
[x_des, xd_des, sigma] = simulate_ds(x);
% Compute optimal gains
[K] = compute_optimal_gains(x_des, sigma);
```
First, simulate_ds simulates an open-loop trajectory from the encoded LMDS and returns not only its expected value but also its variance. Second, compute_optimal_gains solves a risk-sensitive optimal regulation problem around the simulated trajectory. Note that this solution is only locally optimal around the simulated trajectory. These two functions are being recomputed on each time step in a Model Predictive Control fashion so the robot will always be around the locallly optimal solutions.  

### Step 3
We can now inspect the behavior by launching simulations from different parts of the workspace by clicking there. In addition, during the simulation you will now be able to perturb the robot with the mouse to get an idea of the robot's compliance. You should notice that the robot is less compliant in regions with low variance and more compliant in regions with high variance. 

### Step 4
Around the top of the file you will find parameters related to the optimization

```matlab
% Optimal control parameters
                % risk-sensitive parameter -> How uncertainty influences robot control. 
theta = -0.1;    % A negative value 'discounts' uncertainty from the overall cost                
                % more uncertainty -> more compliance

%theta = 0;     % Ignores uncertainty: no influence on robot behavior
                
tracking_precision_weight = 1e4; % desired position tracking precision

time_horizon = 10;
```

To observe the effect of the risk-sensitivity parameter, set it to 0 and compare the resulting behavior with the previous runs. In this case we are ingnoring uncertainty and the compliance is not affected by uncertainty.



# References
1. P.I. Corke, Robotics, Vision & Control, Springer 2011, ISBN 978-3-642-20143-1
2. Khansari Zadeh, S. M. and Billard, A., Learning Stable Non-Linear Dynamical Systems with Gaussian Mixture Models. IEEE Transaction on Robotics, vol. 27, num 5, p. 943-957 [link to pdf](http://lasa.epfl.ch/publications/uploadedFiles/Khansari_Billard_TRO2011.pdf)
3. Kronander, K., Khansari Zadeh, S. M. and Billard, A. (2015) Incremental Motion Learning with Locally Modulated Dynamical Systems. Robotics and Autonomous Systems, 2015, vol. 70, iss. 1, pp. 52-62. [link to pdf](http://lasa.epfl.ch/publications/uploadedFiles/LMDS_els.pdf)
4. Carl Edward Rasmussen, Gaussian Processes for Machine Learning, The MIT Press, 2006. ISBN 0-262-18253-X
5. Medina, J. R., Lorenz, T., and Hirche, S. (2015). Synthesizing Anticipatory Haptic Assistance Considering Human Behavior Uncertainty. Robotics, IEEE Transactions on, 31(1), 180-190.
