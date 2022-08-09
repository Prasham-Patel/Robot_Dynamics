
function T = fkine(S,M,q,frame)
    % your code here
    
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
    T = eye(4);
    if frame == "body"
    T = M;
    end
    [rows, cols] = size(S);
    for i = 1:cols
        T = T*twist2ht(S(:, i), q(i));
    end
    if frame == "space"
    T = T*M;
    end
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
end