function J_a = jacoba(S,M,q)    
    % your code here
    T = fkine(S,M,q,'space');
    J_b = jacobe(S,M,q);
    J_a = T(1:3,1:3)*J_b(4:6,:);
end

