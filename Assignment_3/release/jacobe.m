
function J_b = jacobe(S,M,q)    
    % your code here
    
    % your code here
    [rows, cols] = size(S);
    T = twist2ht(S(:,1), q(1));
        
    J = inverse_adjoint(S(:, 1), T);
    for i = 2:cols
        T = T*twist2ht(S(:,i), q(i));
        J = horzcat(J, inverse_adjoint(S(:, i), T));
    end
   
    T = T*M;
    
    R = [T(1:3, 1:3)]';
    omega_s = -R*[T(1, 4); T(2, 4); T(3, 4)];
    P = [0, -omega_s(3), omega_s(2); 
          omega_s(3), 0, -omega_s(1);
          -omega_s(2), omega_s(1), 0];
    J_b = [R, zeros(3, 3); P*R, R]*J;

end