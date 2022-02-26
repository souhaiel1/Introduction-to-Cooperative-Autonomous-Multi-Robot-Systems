
%--------------- Souhaiel Ben Salem ------------------

% Flocking using the 3 rules of Reynolds

%% Flocking double integrator robots 
function flocking_integrator

% robot model
% p = [x;y]
% v = [vx;vy]
% double integrator
% p_dot = v
% v_dot = u
% define x =[x;y;vx;vy]
% x_dot = [vx;vy;ux;uy]

% robot dynamics function
    function x_dot=f(x,u)
        x_dot=[x(3);x(4);u(1);u(2)];
    end

init;
% sampling time
dt=0.09;
%number of robots
N=30;
% vector of all robot states X=[x1,x2,...,xN]
% random robot initial states
X(1:2,:)=13*randn(2,N);
X(3:4,:)=20*randn(2,N);
% vector of all control inputs U=[u1,u2,...,uN]
U=zeros(2,N);
% adjacency matrix of the fully connected graph ( all-to-all communication):
A = ones(N)-eye(N) ;
% degree matrix of the complete graph ( all-to-all communication):
D = (N-1)*eye(N);
% Laplacien matrix of  the complete graph ( all-to-all communication):
L = D - A ;

%simulation
for t=0:dt:100
    clf();
    axis([-80,80,-80,80]);
    axis square; hold on
    alpha=0.4; 
    beta=0.2; 
    sigma=400; 
    delta=1; 
    Urep_grad=0;
    p=X(1:2,:);
    repulsive_force=zeros(2,N); 
    X_dot=-L*X.' ;
    for i=1:N
       for j=1:N
        if j~=i     
        % control law for robot i
        Urep_grad =(-0.5*(p(:,i)-p(:,j))/norm(p(:,i)-p(:,j))^3);      
        else 
            Urep_grad=0;
        end
         repulsive_force(:,i)= -( Urep_grad) + repulsive_force(:,i) 
        end
        % control law for robot i
        U(:,i)=alpha*X_dot(i,3:4).'+ beta*X_dot(i,1:2).'+sigma*repulsive_force(:,i);
        % update the position of robot i using its dynamics
        X(:,i)=X(:,i)+f(X(:,i),U(:,i))*dt;
        % draw the robots as circles
        plot(X(1,i),X(2,i),'oblack','LineWidth',3)
    end
    drawnow();
end

end