import numpy as np
import matplotlib.pyplot as plt
from sympy import symbols, atan, sqrt, sin, lambdify, pi

# Define symbolic variables
act, L, z = symbols('act L z')

# Define the equation
equation = (L*sqrt(1 - (L*sin(2*atan((2*L*act + sqrt(4*L**2*act**2 + 16*L**2*z**2 - act**4 - 8*act**2*z**2 - 16*z**4))/(4*L*z + act**2 + 4*z**2))) - act)**2/L**2) - z)

# Define values for act and z
act_value = 45
z_value = sqrt(3)/3*(84.497542228927-13.3842608598)

# Substitute act and z with the given values
equation_sub = equation.subs({act: act_value, z: z_value})

# Lambdify the equation to evaluate it numerically
equation_lambdified = lambdify(L, equation_sub, 'numpy')

# Generate a range of L values
L_values = np.linspace(46, 150, 50000)  # Adjust the range and number of points as needed

# Evaluate the equation for each L value
results = equation_lambdified(L_values)

# Plot the results
plt.figure(figsize=(3, 3))
plt.plot(L_values, results)
plt.xlabel('Leg length [mm]')
plt.ylabel('Working radius [mm]')
plt.ylim(bottom=0)
plt.ylim(top=67)
plt.xlim(left=40)
plt.xlim(right=140)
plt.grid(False)
plt.tight_layout()  # Automatically adjust subplot parameters to fit in the figure area

plt.show()
