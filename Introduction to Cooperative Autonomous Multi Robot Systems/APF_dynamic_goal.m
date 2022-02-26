
%--------------- Sohaiel Ben Salem ----------------


%% Navigation with artificial potential fields - nonholonomic vehicle

function navigation_potential

% robot model
% p = [x;y]
% theta = heading angle
% v = speed vector
% dubins' car model
% p_dot = [v*cos(theta);v*sin(theta)]
% v_dot = u1
% theta_dot = u2
% define x = [x;y;v;theta]
% x_dot = [v*cos(theta);v*sin(theta);u1;u2]

% robot dynamics function
    function x_dot=f(x,u)
        x_dot=[x(3)*cos(x(4));
               x(3)*sin(x(4));
               u(1);
               u(2)];
    end


% Normalized potential field gradient vectors
    function draw_field(p_obs,p_goal,v_goal)
        X=xmin:0.2:xmax;
        Y=ymin:0.2:ymax;
        [P1,P2]=meshgrid(X,Y);
        dobs1=P1-p_obs(1);
        dobs2=P2-p_obs(2);
        VX=v_goal(1)-2*alpha*(P1-p_goal(1))+beta*(dobs1)./((dobs1.^2+dobs2.^2).^(3/2));
        VY=v_goal(2)-2*alpha*(P2-p_goal(2))+beta*(dobs2)./((dobs1.^2+dobs2.^2).^(3/2));
        VX=VX./sqrt(VX.^2+VY.^2);
        VY=VY./sqrt(VX.^2+VY.^2);
        quiver(X,Y,VX,VY);
    end

%---------------- Main -------------------
init;
xmin=0;
xmax=15;
ymin=0;
ymax=15;

% obstacle initial position
p_obs=[5;5];
% goal initial position
p_goal0=[0;0];


%  second question
p_goal=[13;13];   

% goal initial speed
v_goal=[0;0];

% sampling time
dt=0.05;

% robot state initial conditions
x=[2;1;1;pi/4];
dobs1=x(1)-p_obs(1);
dobs2=x(2)-p_obs(2);
%simulation
for t=0:dt:20
    clf(); 
    hold on;
    axis([xmin xmax ymin ymax]); 
    axis square;
    alpha=1;
    beta=5;
    p=[x(1) ; x(2)] ;
    p_goal(1)=p_goal0(1)+ (2/3*t+sin(t)^2) ;
    p_goal(2)=p_goal0(2)+(2/3*t+cos(t)^2) ;
    p_goal=[p_goal(1) ;  p_goal(2)] ;
    % repulsive potential function gradient 
    Urep_grad = -0.5*beta*(p-p_obs)/norm(p-p_obs)^3;
    % attractive potential function gradient
    Uatt_grad= 2* alpha*(p-p_goal) ;

    % reference velocity vector
    negative_gradient= -(Uatt_grad + Urep_grad) ; 

    % desired speed vector
    %vref=0;
    vref= norm(negative_gradient) ;
    % desired heading angle
    %thetaref=0;
     thetaref= atan2(negative_gradient(2), negative_gradient(1)) ;
    %control law
    delta=1;
    gamma=5;
    u=[delta*(vref-x(3)); gamma*2*atan(tan(0.5*(thetaref-x(4))))];

    % robot state update using its dynamics
    x=x+f(x,u)*dt;

    % draw the potential field
    draw_field(p_obs,p_goal,v_goal);
    % draw the robot
    draw_tank(x([1,2,4]),'red',0.1);
    % draw the goal position
    plot(p_goal(1),p_goal(2),'ogreen','LineWidth',3);
    % draw the obstacle position
    plot(p_obs(1),p_obs(2),'oblack','LineWidth',3);
    drawnow();
end
end
%% Consensus simple integrator robots

function STUDENTS_Consensus_simple_integrator

% robot model
% p = [x;y]
% simple integrator
% p_dot = u
% define x =[x;y]
% x_dot = [ux;uy]

% robot dynamics function
    function x_dot=f(x,u)
        x_dot=[u(1);u(2)];
    end

init;
% sampled time
dt=0.2;
%number of robots
N=50;
% vector of all robot states X=[x1,x2,...,xN]
% random robot initial states
X=25*randn(2,N);
% vector of all control inputs U=[u1,u2,...,uN]
U=zeros(2,N);

%simulation
for t=0:dt:100
    
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on

    % control parameters
    % setting alpha for discrete time version
    alpha=0.5;
    beta=1;
    
    % laplas matrix for all the robots
    L = -1*ones(N);
    for i=1:N
        L(i,i) = N-1;
    end
    
    % control law for all the robots at once we then use the U matrix for
    % each robot in the loop below
    U=alpha*transpose(-L*X.');
    
    for i=1:N
        
        % update the position of robot i using its dynamics
        X(:,i)=X(:,i)+f(X(:,i),U(:,i))*dt;
        % draw the robots as circles
        plot(X(1,i),X(2,i),'oblack','LineWidth',3)
    end
    drawnow();
end

end


