
function exercise_4()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% open parameters %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% hyper-parameters for gaussian process
% these can be learned from data but we will use predetermined values here
ell = 0.1; % lengthscale. bigger lengthscale => smoother, less precise ds
sf = 10; % signal variance 
sn = 1; % measurement noise 


% Optimal control parameters
                % risk-sensitive parameter -> How uncertainty influences robot control. 
theta = -0.1;    % A negative value 'discounts' uncertainty from the overall cost                
                % more uncertainty -> more compliance

theta = 0;     % Ignores uncertainty: no influence on robot behavior
                
tracking_precision_weight = 1e4; % desired position tracking precision

time_horizon = 10;

options.objective = 'likelihood';    % 'likelihood'
nb_gaussians = 2;
nb_demo = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global pertForce;

pertForce = [0;0];

close all
% set up a simple robot and a figure that plots it
robot = create_simple_robot();
fig = initialize_robot_figure(robot);
%fig = figure(1);clf;
%robot.plot([0,1])
disp('In this exercise you will peroform very simple record and replay of a demonstrated trajectory.')
disp('You can see the robot in the figure. Give a demonstration of a trajectory in its workspace')
% get a demonstration

% select the number of knots for the spline representation
nb_knots = 10;
% select how many points we should use to represent each demonstration
nb_clean_data_per_demo = 50;
Data = [];
target = zeros(2,1);
hpp = [];
for i=1:nb_demo
    [data,hp] = get_demonstration(fig,0);
    nb_data = size(data,2);
    skip = floor(nb_data/nb_knots);
    knots = data(:,1:skip:end);
    ppx = spline(knots(3,:),knots(1:2,:));
    % % and get first derivative
    ppxd = differentiate_spline(ppx);
    tt = linspace(knots(3,1), 0.9*knots(3,end), nb_clean_data_per_demo);
    pos = ppval(ppx,tt);
    target = target+pos(:,end);
    pos = pos - repmat(pos(:,end), 1, nb_clean_data_per_demo);
    %pos(:,end)
    vel = ppval(ppxd,tt);
    %vel(:,end) = zeros(2,1);
    Data = [Data, [pos;vel]];
    hpp = [hpp, hp];
end
delete(hpp);
target = target/nb_demo;
plot(Data(1,:)+target(1), Data(2,:)+target(2), 'r.','markersize',20);
plot(target(1), target(2),'k.','markersize',50);

% learn SEDS model
options.tol_mat_bias = 10^-6; % A very small positive scalar to avoid
                              % instabilities in Gaussian kernel [default: 10^-1                             
options.display = 1;          % An option to control whether the algorithm
                              % displays the output of each iterations [default: true]                            
options.tol_stopping=10^-6;  % A small positive scalar defining the stoppping
                              % tolerance for the optimization solver [default: 10^-10]
options.max_iter = 1000;       % Maximum number of iteration for the solver [default: i_max=1000]
                        
[Priors_0, Mu_0, Sigma_0] = initialize_SEDS(Data,nb_gaussians); %finding an initial guess for GMM's parameter
[Priors Mu Sigma]=SEDS_Solver(Priors_0,Mu_0,Sigma_0,Data,options); %running SEDS optimization solver
ds = @(x) GMR(Priors,Mu,Sigma,x,1:2,3:4);
%hs = plot_ds_model(fig, ds, target);
qi = simple_robot_ikin(robot,data(1:2,1));
robot.animate(qi);
disp('To improve the accuracy, we use GP-MDS to locally reshape the DS around the demonstrations')

% get data for learning the reshaping function
% each demonstration will be converted
% from series of pos, vel to series of pos, angle, speed_factor
lmds_data = [];
for i =1:nb_demo
    dsi = 1+(i-1)*nb_clean_data_per_demo; % demonstration start index
    dei = i*nb_clean_data_per_demo; % demonstration end index
    lmds_data = [lmds_data, generate_lmds_data_2d(Data(1:2,dsi:dei),Data(3:4,dsi:dei),ds(Data(1:2,dsi:dei)),0.05)];
end


% we pack the hyper paramters in logarithmic form in a structure
hyp.cov = log([ell; sf]);
hyp.lik = log(sn);
% for convenience we create a function handle to gpr with these hyper
% parameters and with our choice of mean, covaraince and likelihood
% functions. Refer to gpml documentation for details about this. 
gp_handle = @(train_in, train_out, query_in) gp(hyp, ...
    @infExact, {@meanZero},{@covSEiso}, @likGauss, ...
    train_in, train_out, query_in);
% we now define our reshaped dynamics
% go and have a look at gp_mds_2d to see what it does
reshaped_ds = @(x) gp_mds_2d(ds, gp_handle, lmds_data, x);
%delete(hs); % delete the seds streamlines
hs = plot_ds_model(fig, reshaped_ds, target); % and replace them with the reshaped ones
% to understand where the gp has infuence, it is useful to plot its
% variance 
hv = plot_gp_variance_2d(fig, gp_handle, lmds_data(1:2,:)+repmat(target, 1,size(lmds_data,2)));

% start simulation
dt = 0.005;
while 1
    disp('Select a starting point for the simulation.')
    disp('Once the simulation starts you can perturb the robot with the mouse to get an idea of its compliance.')
    try
        xs = get_point(fig);
        qs = simple_robot_ikin(robot, xs);
        qd = [0,0];
        robot.animate(qs);
        simulation_opt_control(qs);
    catch
        disp('could not find joint space configuration. Please choose another point in the workspace.')
    end
end

    function simulation_opt_control(q)
        t = 0;
        qd = [0,0];
        ht = [];
        
        % Set perturbations with the mouse
        set(gcf,'WindowButtonDownFcn',@startPerturbation);
        
        while(1)
            % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:2,4);
            xd = robot.jacob0(q)*qd';
            xd = xd(1:2);
            
            xd_ref = reshaped_ds(x-target);%reference_vel(t);
            % put lower bound on speed, just to speed up simulation
            th = 1.0;
            if(norm(xd_ref)<th)
                xd_ref = xd_ref/norm(xd_ref)*th;
            end
            xdd_ref = -(xd - xd_ref)/dt*0.5;
            
            % Get open loop trajectory from the DS
            [x_des, xd_des, sigma] = simulate_ds(x);
            % Compute optimal gains
            [K] = compute_optimal_gains(x_des, sigma);
            % compute cartesian control
            u_cart = 0;
            % feedforward term
            u_cart = u_cart + simple_robot_cart_inertia(robot,q)*xdd_ref;
            % optimal damping term
            u_cart = u_cart - K(:,3:4,1)*(xd_des(:,1) - xd);
            % external perturbations with the mouse
            u_cart = u_cart + pertForce;
            % compute joint space control
            u_joint = robot.jacob0(q)'*[u_cart;zeros(4,1)];
            % apply control to the robot
            qdd = robot.accel(q,qd,u_joint')';
            % integrate one time step
            qd = qd+dt*qdd;
            q = q+qd*dt+qdd/2*dt^2;
            t = t+dt;
            if (norm(x - target)<0.1)
                break
            end
            robot.delay = dt;
            robot.animate(q);
            
            ht = [ht, plot(x(1), x(2), 'm.', 'markersize',20)];
        end
      
        delete(ht);
    end

    %% Computes open-loop trajectory of mean+variances until the origin
    function [xdes, xddes, sigma] = simulate_ds(x)
        xdes(:,1) = x;
        xddes(:,1) = reshaped_ds(x-target);
        sigma(:,:,1) = zeros(2);
        
        for traj_index=1:time_horizon
            % compute state of end-effector
            [x_dot,var] = reshaped_ds(x-target);%reference_vel(t);
            
            % put lower bound on speed, just to speed up simulation
            th = 1.0;
            if(norm(x_dot)<th)
                x_dot = x_dot/norm(x_dot)*th;
            end
            x = x + x_dot*dt;
            
            xdes(:,traj_index) = x;
            xddes(:,traj_index) = x_dot;
            sigma(:,:,traj_index) = var*dt;
        end
    end

    %% Computes optimal tracking control by minimizing cost
    % J = \sum_{k=1}^{T} (x_k - x_des_k)^T Q (x_k - x_des_k) + u_k^T R u_k
    function [K] = compute_optimal_gains(x_des, sigma)
        
        % Q/R parameters
        Q = tracking_precision_weight * eye(4);
        R = 1 * eye(2);

        % Quadratic coefficient at time horizon
        W = Q;
        
        % Ricatti recursion
        for k=time_horizon-1:-1:1
            % Robot cartesian inertia and damping (Approximate around desired trajectory and discard DS derivatives)
            M = simple_robot_cart_inertia(robot,simple_robot_ikin(robot, x_des(:,k)));
            D = zeros(2);

            % Discrete-time dynamics 
            A = eye(4) + dt*[zeros(2) eye(2);zeros(2) -(M\D)];
            B = dt*[zeros(2); inv(M)];
            
            % Risk sensitive expectation
            W_risk = (eye(4) - theta*W* blkdiag(zeros(2), sigma(:,:,k)) )\W;
            
            % Standard LQR
            H = (R + B'*W_risk*B);
            G = (B'*W_risk*A);
            K(:,:,k) = -H\G;
            W = Q + A'*W_risk*A + K(:,:,k)'*H*K(:,:,k) + K(:,:,k)'*G + G'*K(:,:,k);
        end
    end

    %% Perturbations with the mouse
    function startPerturbation(~,~)
        motionData = [];
        set(gcf,'WindowButtonMotionFcn',@perturbationFromMouse);
        x = get(gca,'Currentpoint');
        x = x(1,1:2)';
        hand = plot(x(1),x(2),'r.','markersize',20);
        hand2 = plot(x(1),x(2),'r.','markersize',20);
        set(gcf,'WindowButtonUpFcn',@(h,e)stopPerturbation(h,e));

        function stopPerturbation(~,~)
            delete(hand)
            delete(hand2)
            set(gcf,'WindowButtonMotionFcn',[]);
            set(gcf,'WindowButtonUpFcn',[]);
            pertForce = [0;0];
        end


        function ret = perturbationFromMouse(~,~)
            x = get(gca,'Currentpoint');
            x = x(1,1:2)';
            motionData = [motionData, x];
            pertForce = 10*(motionData(:,end)-motionData(:,1));
            ret=1;
            delete(hand2)
            hand2 = plot([motionData(1,1),motionData(1,end)],[motionData(2,1),motionData(2,end)],'-r');
        end
    end
        


end



