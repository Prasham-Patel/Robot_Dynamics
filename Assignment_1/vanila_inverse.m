function Delta_theta = vanila_inverse(J, Vd, V0)
    Delta_theta = pinv(J)*(Vd - V0);
end