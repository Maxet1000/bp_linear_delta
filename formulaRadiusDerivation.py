import sympy as sp
import numpy as np

# Define symbols
# note that z is (z-c) in the paper, the correction for the size of the end effector is ignored here to reduce the
# required time to find a solution
L, z, R, alpha, beta, act= sp.symbols('L z R alpha beta act')

# Define the equations
eq1 = L*sp.sin(alpha) - L*sp.sin(beta) + act
eq2 = L*sp.cos(alpha) - 2*R - L*sp.cos(beta)
eq3 = L*sp.cos(alpha) - R - sp.sqrt(3)/3*(z)

solutions = sp.solve((eq1, eq2, eq3), (alpha, beta, R))

print("Symbolic solutions:")
print(solutions)

# Calculate the Numeric solution, the only equation that returns a positive value is the correct one
L = 70
z = 84.497542228927 - 13.3842608598
act = 45

for sol in solutions:
    equation = sol[2]
    print(equation)

    expression = str(equation)
    expression = expression.replace('sqrt', 'np.sqrt')
    expression = expression.replace('sin', 'np.sin')
    expression = expression.replace('atan', 'np.arctan')
    answer = eval(expression)
    print(answer)
