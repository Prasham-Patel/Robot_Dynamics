function Delta_theta = gradient_inverse(J, Vd, V0, alpha)
    Delta_theta = alpha*J'*(Vd - V0);
end