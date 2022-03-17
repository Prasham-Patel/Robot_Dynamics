function twist_inB = adjoint(V,T)
    % your code here
    R = T(1:3, 1:3);
    P = T(1:3, 4);
    b_P = [0, -P(3), P(2); P(3), 0, -P(1); -P(2), P(1), 0];
    
    twist_inB = [R, zeros(3,3); b_P*R, R]*V;
end


