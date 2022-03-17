function T = twist2ht(S,theta)

% T = twist2ht(S, theta) converts twist in to
% homogeneous transformation matrix
% given twist 1x6 and a scalar theta which is angle of rotation of joint

    omega = [S(1), S(2), S(3)];
    v = [S(4), S(5), S(6)]';
    w = [0, -omega(3), omega(2); omega(3), 0, -omega(1); -omega(2), omega(1), 0];
    R = eye(3) + (w^2)*(1 - cos(theta)) + w*sin(theta);
    P = (eye(3)*theta + (w^2)*(theta - sin(theta)) + w*(1 - cos(theta)))*v;
    T = [R, P; 0, 0, 0, 1];
end