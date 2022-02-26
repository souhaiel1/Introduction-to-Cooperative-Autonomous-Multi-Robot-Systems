%---------- Computer lab Autonomous Systems ----------
%-------------- Master MARS-----------------
%-------- Souhaiel Ben Salem ----------

%% Consensus simple integrator robots
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
dt=0.001;
%number of robots
N=50;
% vector of all robot states X=[x1,x2,...,xN]
% random robot initial states
X=25*rand(2,N);
J=sum(X,2);
fprintf('the average x position of the initial state: %f\n', J(1)/N)
fprintf('the average y position of the initial state: %f\n', J(2)/N)
% x_avg=(1/N)*(sum(X,2)(1))
% y_avg=(1/N)*(sum(X,2)(2))


% vector of all control inputs U=[u1,u2,...,uN]
U=zeros(2,N);
%% Question 1 
% The adjacency matrix of a complete ( fully connected graph) with N nodes
A = ones(N)-eye(N) ;

%simulation
for t=0:dt:10
    clf();
    axis([-60,60,-60,60]);
    axis square; hold on

    % control parameters
    alpha=1;
    beta=1;
    % Comsencus algorithm
    for i=1:N
        % control law for robot i
        % Here we used the analytical expression of consensus algorithm  
        U(:,i)= -A(:,i).'*(X(:,i)-X).';
        % update the position of robot i using its dynamics
        X(:,i)=X(:,i)+f(X(:,i),U(:,i))*dt;
        %%
        % draw the robots as circles
        plot(X(1,i),X(2,i),'oblack','LineWidth',3)
    end
    drawnow();
end

end