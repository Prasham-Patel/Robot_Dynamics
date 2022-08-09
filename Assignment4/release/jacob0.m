function J = jacob0(S,q)
    % your code here
    
    % If necessary, you calculate the homogeneous transformation associated with a twist V and displacement omega with:
    % T = twist2ht(V,omega);
    T = eye(4);
    J = zeros(6,length(q));
    for i = 1:length(q)
        Si = S(:,i);
        if i~=1
            T = T*twist2ht(S(:,i-1),q(i-1));
        end
        p = T(1:3,4);
        p_box = [0,-p(3),p(2);p(3),0,-p(1);-p(2),p(1),0];
        Adj_T = [T(1:3,1:3) zeros(3,3); (p_box*T(1:3,1:3)) T(1:3,1:3)];
        J(:,i) = Adj_T*S(:,i);
    end
    % You can also calculate the adjoint transformation of a twist V w.r.t. a homogeneous transformation matrix T with:
    % adjoint(V,T)
end

