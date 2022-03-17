- Run the ik.m file.

- qList contains all the angle configurations for the corresponding position.

- 2 inverse kinematics method are applied: 
	# Gradient Method
	# Newton-Raphson

- code for all 3 methods for inverse kinematics are code:
	# Newton-Raphson (vanilla_inverse.m)
	# Levenberg-Marquadt (damped_inverse.m)
	# Gradient Method (gradient_inverse.m)

- jacob0.m contains function to calculate jacobian.

- fkin.m contains function for forward kinematics.

- adjoint.m contains function to calculate adjoint.

- also works for resolution less than 0.01 mm