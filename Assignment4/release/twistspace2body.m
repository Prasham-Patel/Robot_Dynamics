function V_b = twistspace2body(V_s,T)
    % your code here
    w_s = V_s(1:3) 
    v_s = V_s(4:6)
    R = T(1:3,1:3);
    p = T(1:3,4);
    w_b = R'*w_s;
    v_b = box_func(-R'*p)*R'*w_s+R'*v_s;
    V_b = [w_b;v_b];
end

