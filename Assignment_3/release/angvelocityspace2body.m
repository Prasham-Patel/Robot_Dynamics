function omega_b = angvelocityspace2body(omega_s,R)
    Ws = [0, -omega_s(3), omega_s(2); 
          omega_s(3), 0, -omega_s(1);
          -omega_s(2), omega_s(1), 0];
    Wb = inv(R)*Ws*R;
    omega_b = [-Wb(2, 3); Wb(1, 3); -Wb(1, 2)];
end
