function theta1 = theta_star(stateNH)

xNI = stateNH(1:3);
eta = stateNH(4:5);

options = optimset('fmincon');
options = optimset(options,'TolX', 1e-6, 'MaxIter', 150, 'Display', 'off', 'Algorithm', 'sqp');

theta1Init = 0;

thetaMin = - pi;
thetaMax = pi;

theta1 = fmincon(@(theta) Fc(xNI, eta, theta), theta1Init,[],[],[],[],thetaMin,thetaMax,[], options);

    function Fval = Fc(x, eta, theta)
        %                                                   3
        %                                               |x |
        %                   4     4                     | 3|
        %   F(x; theta) = x   +  x   + ----------------------------------------
        %                   1     2
        %                               /                                     \ 2
        %                              | x cos theta + x sin theta + sqrt|x |  |
        %                               \ 1             2                | 3| /
        %
        %                                  \_______________  ______________/
        %                                                  \/
        %                                                 sigma~
        %
        %
        %                              T
        %   F (x) = F(x; theta) + 1/2 z z
        %    c
        
        sigma_tilde = x(1)*cos(theta) + x(2)*sin(theta) + sqrt(abs(x(3)));
        
        F = x(1)^4 + x(2)^4 + abs(x(3))^3 / sigma_tilde^2;
        
        z = eta - kappa_tilde(x, theta);
        
        Fval = F + 1/2 * (z' * z);
        
        function kappaVal = kappa_tilde(x, theta) 
            kappaVal = [0; 0];
            
            G = zeros(3, 2);
            
            G(:, 1) = [1; 0; x(2)];
            
            G(:, 2) = [0; 1; -x(1)];
            
            zeta = zeta_tilde(x, theta);
            
            kappaVal(1) = - abs( dot( zeta, G(:, 1) ) )^(1/3) * sign( dot( zeta, G(:, 1) ) );
            
            kappaVal(2) = - abs( dot( zeta, G(:, 2) ) )^(1/3) * sign( dot( zeta, G(:, 2) ) );
            
            function zetaVal = zeta_tilde(x, theta)               
                zetaVal = zeros(3, 1);
                
                sigma_tilde = x(1)*cos(theta) + x(2)*sin(theta) + sqrt(abs(x(3)));
                
                nabla_F = zeros(3,1);
                
                nabla_F(1) = 4*x(1)^3 - 2 * abs(x(3))^3 * cos(theta)/sigma_tilde^3;
                
                nabla_F(2) = 4*x(2)^3 - 2 * abs(x(3))^3 * sin(theta)/sigma_tilde^3;
                
                nabla_F(3) = ( 3*x(1)*cos(theta) + 3*x(2)*sin(theta) + 2*sqrt(abs(x(3))) ) * x(3)^2 * sign(x(3)) / sigma_tilde^3;
                
                zeta = nabla_F;
               
            end
        end
    end
end