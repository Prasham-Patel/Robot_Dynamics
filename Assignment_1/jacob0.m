function J = jacob0(S,q) 
    [~, cols] = size(S);
    T = twist2ht(S(:,1), q(1));
    J = adjoint(S(:, 1), T);
    for i = 2:cols
        T = T*twist2ht(S(:,i), q(i));
        J = horzcat(J, adjoint(S(:, i), T));
    end
end