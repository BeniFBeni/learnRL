function [A, B, C, D, x0est] = mySSest(ys, us, dt, modelOrder, Ainit, Binit, Cinit, Dinit)
% ys and us are (signal dim) × (# of data samples)
% Returns state-space estimated model of modelOrder

dataset = iddata(ys', us', dt);

try
    if nargin > 4   % Based on initial system
        SSinit = idss(ss(Ainit, Binit, Cinit, Dinit, dt));
        
%         SSinit.Structure.C.Free = false;    % No alteration of output matrix
        SSinit.Structure.B.Free = false;    % No alteration of input matrix
        
        [mySS, x0est] = ssest(dataset, SSinit);
    else
        [mySS, x0est] = ssest(dataset, modelOrder, 'Ts', dt);
    end
catch
    warning('Error: state estimator failed -- try adjusting estBufferSize, modelOrder, modelUpdDelay in init.m'); 
    
    [l, ~] = size(us);
    [p, ~] = size(ys);
    
    mySS = rss(modelOrder, p, l);
    x0est = zeros(modelOrder, 1);
  
end

A = mySS.A;
B = mySS.B;
C = mySS.C;
D = mySS.D;

end

