function T = twist2ht(S,theta)
    % your code here
    omega = S(1:3);
    V = S(4:6);
    omega_box = [0,-omega(3),omega(2);omega(3),0,-omega(1);-omega(2),omega(1),0];
    R = eye(3) + sin(theta)*omega_box + (1-cos(theta))*(omega_box*omega_box);
    d = ((eye(3)*theta)+((1-cos(theta))*omega_box)+((theta-sin(theta))*(omega_box*omega_box)))*V;
    T = [R d;zeros(1,3) 1];
end

