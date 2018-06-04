
function exercise_1()
close all
% set up a simple robot and a figure that plots it
robot = create_simple_robot();
fig = initialize_robot_figure(robot);
%fig = figure(1);clf;
%robot.plot([0,1])
disp('In this exercise you will peroform very simple record and replay of a demonstrated trajectory.')
disp('You can see the robot in the figure. Give a demonstration of a trajectory in its workspace')
% get a demonstration
data = get_demonstration(fig);
disp('Its convenient to represent the collected data with interpolating polynomials, e.g. splines. This allows limit the complexity as well as guarantee smooth trajectories.')
nb_knots = 10;
disp(sprintf('you now see a spline with %d knots in the figure.', nb_knots));
% fit a spline to the demonstration
nb_data = size(data,2);
skip = floor(nb_data/nb_knots);
knots = data(:,1:skip:end);
ppx = interp1(knots(3,:)',knots(1:2,:)','spline','pp');
% and get first and second derivative
ppxd = differentiate_spline(ppx);
ppxdd = differentiate_spline(ppxd);

% lets plot the demonstrated trajectory
plot(knots(1,:), knots(2,:), 'b+');
ref_traj = ppval(ppx,linspace(knots(3,1),knots(3,end),1000))';
plot(ref_traj(1,:),ref_traj(2,:),'k-');

% find an initial joint configuration for the start point
%qi = robot.ikine(transl(knots(1,1), knots(2,1),0.0),[0.2,0.2],[1,1,0,0,0,0]);
qi = simple_robot_ikin(robot, knots(1:2,1));
robot.animate(qi);
% simulate tracking of the trajectory in the absence of perturbations
% We will use a cartesian impedance controller to track the motion
% we need to define stiffness and damping values for our controller

K = eye(2)*1000;
D = eye(2)*5;
% start simulation
dt = 0.005;
% simulation from same start point
disp('The robot will be able to perform the task when starting from the same starting location as the demonstration.')
disp('press enter to continure..')
pause
simulation(qi);
% simulation from different starting point
while 1
    disp('Now we imagine the robot starts the task from a different location. click on a departure point in the robot workspace.')
    try
        xs = get_point(fig);
        qs = simple_robot_ikin(robot,xs);
        robot.animate(qs)
        disp('The simple time-dependent reference trajectory approach cannot deal with this situation. Press enter to see what happens..')
        pause
        simulation(qs);
    catch
        disp('could not find joint space configuration. Please choose another point in the workspace.')
    end
end

%simulation(qi,1);
    function simulation(q)
        t = knots(3,1);
        qd = [0,0];
        x_ref = ppval(ppx, t);
        h = plot(x_ref(1),x_ref(2),'go');
        ht = [];
        while(1)
            % compute state of end-effector
            x = robot.fkine(q);
            x = x(1:2,4);
            xd = robot.jacob0(q)*qd';
            xd = xd(1:2);
            %eig(cart_inertia(robot, q))
            
            % compute our time-dependent refernce trajectory
            x_ref = ppval(ppx, t);%reference_pos(t);
            xd_ref = ppval(ppxd, t);%reference_vel(t);
            xdd_ref = ppval(ppxdd, t);%reference_acc(t);
            xdd_ref = xdd_ref-0.5*(xd-xd_ref);
            % compute cartesian control
            u_cart = -K*(x-x_ref) - D*(xd-xd_ref);
            % feedforward term
            u_cart = u_cart + simple_robot_cart_inertia(robot,q)*xdd_ref;
            % compute joint space control
            u_joint = robot.jacob0(q)'*[u_cart;zeros(4,1)];
            % apply control to the robot
            qdd = robot.accel(q,qd,u_joint')';
            % integrate one time step
            qd = qd+dt*qdd;
            q = q+qd*dt+qdd/2*dt^2;
            t = t+dt;
            if (norm(x_ref - knots(1:2,end))<0.01)
                break
            end
            robot.delay = dt;
            robot.animate(q);
            set(h,'XData',x_ref(1));
            set(h,'YData',x_ref(2));
            ht = [ht, plot(x(1), x(2), 'm.','markersize',10)];
        end
        delete(h);
        delete(ht);
    end

end


