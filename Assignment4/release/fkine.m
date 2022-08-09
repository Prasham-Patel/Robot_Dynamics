function T = fkine(S,M,q,frame)
    % your code here
    if strcmp(frame,'space') == 1
        T = eye(4);
        for i = 1:length(q)
            T = T*twist2ht(S(:,i),q(i));
        end
        T = T*M;
    else
        T = M;
        for i = 1:length(q)
            T = T*twist2ht(twistspace2body(S(:,i),twist2ht(S(:,i),q(i))),q(i));
        end
    end
    
    % If needed, you can convert twists to homogeneous transformation matrices with:
    % twist2ht(S(i),q(i));
end

