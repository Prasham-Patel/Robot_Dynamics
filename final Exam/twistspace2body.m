function V_b = twistspace2body(V_s,T)
    
    R = [T(1:3, 1:3)]';
    omega_s = -R*[T(1, 4); T(2, 4); T(3, 4)];
    P = [0, -omega_s(3), omega_s(2); 
          omega_s(3), 0, -omega_s(1);
          -omega_s(2), omega_s(1), 0];
    V_b = [R, zeros(3, 3); P*R, R]*V_s;
end