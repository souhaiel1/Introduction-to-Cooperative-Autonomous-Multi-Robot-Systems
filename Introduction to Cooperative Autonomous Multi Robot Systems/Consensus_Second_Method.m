
%-------- Souhaiel Ben Salem ----------

% Consensus using the Laplacian matrix

%% Question 2
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
dt=0.0005;
%number of robots
N=50;
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
% Laplacian matrix of  the complete graph ( all-to-all communication):
L = D - A ;


%simulation
for t=0:dt:10
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on

    % control parameters
    alpha=1;
    beta=1;
    
    % Comsencus algorithm
    X_dot=-L*X.' ;
    for i=1:N
        % control law for robot i
        U(:,i)= X_dot(i,:).' ;
        % update the position of robot i using its dynamics
        X(:,i)=X(:,i)+f(X(:,i),U(:,i))*dt;
        % draw the robots as circles
        plot(X(1,i),X(2,i),'oblack','LineWidth',3)
    end
    drawnow();
end

end