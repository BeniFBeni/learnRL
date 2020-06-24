function x0 = myFindInitState(A, B, C, D, us, ys, dt)

dataset = iddata(ys', us', dt);

% sys = ss(A, B, C, D, dt);
% sys = idss(sys);

[p, n] = size(C);

sys = idss(A,B,C,D, zeros(n,p), zeros(n,1), dt);

x0 = findstates(sys, dataset);

end

