function T = fkine(S,M,q)
    % your code here
    T = twist2ht(S(:, 1), q(1));
    [~, cols] = size(S);
    for i = 2:cols
        T = T*twist2ht(S(:, i), q(i));
    end
    T = T*M;
end