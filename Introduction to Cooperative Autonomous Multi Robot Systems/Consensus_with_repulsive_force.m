
%-------- Souhaiel Ben Salem ----------

% Adding a repulsive force among the robots

% Consensus simple integrator robots
function consensus_integrator

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
dt=0.0025;
%number of robots
N=20;
% vector of all robot states X=[x1,x2,...,xN]
% random robot initial states
X=50*rand(2,N);
J=sum(X,2);
fprintf('the average x position of the initial state: %f\n', J(1)/N)
fprintf('the average y position of the initial state: %f\n', J(2)/N)

% x_avg=(1/N)*(sum(X,2)(1))
% y_avg=(1/N)*(sum(X,2)(2))


% vector of all control inputs U=[u1,u2,...,uN]
U=zeros(2,N);
% adjacency matrix of the fully connected graph ( all-to-all communication):
A = ones(N)-eye(N) ;
% degree matrix of the complete graph ( all-to-all communication):
D = (N-1)*eye(N);
% Laplacien matrix of  the complete graph ( all-to-all communication):
L = D - A ;

 
%simulation
for t=0:dt:10
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on
    
    % control parameters
    beta=700;
    p=X;
    Urep_grad=0;
    repulsive_force=zeros(2,N); 
    % Comsencus algorithm
    X_dot=-L*X.' ;
    for i=1:N
        for j=1:N
        if j~=i     
        % repilsive gradient
        Urep_grad =(-0.5*beta*(p(:,i)-p(:,j))/norm(p(:,i)-p(:,j))^3);      
        else 
            Urep_grad=0;
        end
         repulsive_force(:,i)= -( Urep_grad) + repulsive_force(:,i) 
        end
        U(:,i)= X_dot(i,:).'+ repulsive_force(:,i) ;
        % update the position of robot i using its dynamics
        X(:,i)=X(:,i)+f(X(:,i),U(:,i))*dt;
        % draw the robots as circles
        plot(X(1,i),X(2,i),'oblack','LineWidth',3)
    end

    drawnow();
end

end