function Delta_theta = damped_inverse(J, Vd, V0, lambda)
    [rows, ~] = size(J);
    Delta_theta = J'*pinv(J*J' + lambda^2*eye(rows))*(Vd - V0);
end