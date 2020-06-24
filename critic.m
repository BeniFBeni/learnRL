function [W, Jc] = critic(Wprev, Winit, U, Y, S, R, M, gamma, Wmin, Wmax, critStruct)
%critic  Critic part of predictive optimal controller (see also optCtrl)
%
%   --- Algorithm ---
%
%       Controller seeks to minimize cost
%
%                   N
%           J = sum     r
%                   1
%
%           where (running cost)
%
%               r = y' * S * y + u' * R * u
%
%
%           and N horizon length
%
%          In RL/ADP, N is interpreted as infinity and so, e.g., in Q-learning, r is substituted for Q-function approximate
%
%          Mode
%             1 - model-predictive control (MPC)
%             2 - MPC with estimated model, a.k.a. adaptive MPC, or AMPC
%             3 - RL/ADP (as stacked Q-learning with horizon N) using true model for prediction
%             4 - RL/ADP (as stacked Q-learning with horizon N) using estimated model for prediction 
%             5 - RL/ADP (as N-step roll-out Q-learning) using true model for prediction
%             6 - RL/ADP (as N-step roll-out Q-learning) using estimated model for prediction 
% 
%             If N = 1, methods are model-free
%
% This function is the critic part of RL. It seeks Q-function approximate as
%
%       Q = W'*phi( y, u )
%
% where W is critic parameters, phi is regressor
%
% W is found by minimizing
%
%                 1       M    2
%           Jc = --   sum     e
%                 2       1
%
%           where (temporal error)
%
%               e = W'*phi( y_prev, u_prev ) - gamma * Wprev'*phi( y_next, u_next ) - r( y_prev, u_prev )
%
%           and M critic stack size
%
%           y's and u's are taken from buffers Y, U resp.
%
% This routine is purely data-driven  
%
% Author: P. Osinenko, 2020
% Contact: p.osinenko@gmail.com

%% Initialization

options = optimset('fmincon');
% options = optimset(options,'TolX', 1e-7, 'TolFun', 1e-7, 'MaxIter', 300, 'MaxFunEvals', 5000, 'Display', 'iter', 'Algorithm', 'sqp');
options = optimset(options,'TolX', 1e-7, 'TolFun', 1e-7, 'MaxIter', 300, 'MaxFunEvals', 5000, 'Display', 'off', 'Algorithm', 'sqp');

%% Critic

[W, Jc] = fmincon(@(W) costFncCritic(W, Y, U, Wprev), Winit, [],[],[],[], Wmin, Wmax, [], options);

%% Critic cost function

    function Jc = costFncCritic(W, Y, U, Wprev)
        
        Jc = 0;
        
        % Buffers are assumed to be filled from right to left
        for k = M:-1:2
            y_prev = Y(:, k-1);
            y_next = Y(:, k);
            u_prev = U(:, k-1);
            u_next = U(:, k);
            
            % Running cost
            r = y_prev' * S * y_prev + u_prev' * R * u_prev;
            
            % Temporal difference
            e =  W'*phi( y_prev, u_prev ) - gamma * Wprev'*phi( y_next, u_next ) - r;  
 
            Jc = Jc + 1/2 * e^2;
        end
        
        function y = uptria2vec(X)
            % Convert upper triangular square sub-matrix to a column vector
            
            [n, ~] = size(X);
            
            y = zeros(n*(n+1)/2, 1);
            
            j = 1;
            for ii = 1:n
                for jj = ii:n
                    y(j) = X(ii, jj);
                    j = j + 1;
                end
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Regressor (RL/ADP)
        function phiVal = phi(y, u)
            
            z = [y; u];
            
            if critStruct == 1 % Quadratic-linear approximator
                phiVal = [ uptria2vec( kron(z, z') ); z ];
                
            elseif critStruct == 2 % Quadratic approximator
                phiVal = uptria2vec( kron(z, z') );
                
            elseif critStruct == 3 % Quadratic approximator, no mixed terms
                phiVal = z.*z;
                
            elseif critStruct == 4 % W(1) y(1)^2 + ... W(p) y(p)^2 + W(p+1) y(1) u(1) + ... W(...) u(1)^2 + ...
                phiVal = [y.^2; kron(y, u) ;u.^2];
            end
            
        end
        
    end
end