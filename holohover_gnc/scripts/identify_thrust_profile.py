import os
import numpy as np
import pandas as pd
import cvxpy as cp
import matplotlib.pyplot as plt


def poly_string(coeff):
    s = str(coeff[0])
    for i in range(1, len(coeff)):
        if coeff[i] > 0:
            s += ' + ' + str(coeff[i])
        else:
            s += ' - ' + str(-coeff[i])
        s += ' * x ** ' + str(i)
    return s

def main():
    data = pd.read_csv(os.path.join(os.path.dirname(__file__), 'thrust_data.csv'), header=None).to_numpy()

    print(f'Max Thrust: {data[-1, 0]} mN')

    signal_to_thrust_deg = 3
    thrust_to_signal_deg = 6

    signal_to_thrust_coeff = cp.Variable(signal_to_thrust_deg + 1)
    A = np.column_stack([data[:, 1] ** i for i in range(signal_to_thrust_deg + 1)])
    b = data[:, 0]
    objective = cp.Minimize(cp.sum_squares(A @ signal_to_thrust_coeff - b))
    constraints = [
        A[0, :] @ signal_to_thrust_coeff == b[0],
        A[-1, :] @ signal_to_thrust_coeff == b[-1]
    ]
    prob = cp.Problem(objective, constraints)
    result = prob.solve(solver=cp.MOSEK)
    print(f'Signal to Thrust coefficients: {signal_to_thrust_coeff.value}')
    print(poly_string(signal_to_thrust_coeff.value))

    thrust_to_signal_coeff = cp.Variable(thrust_to_signal_deg + 1)
    A = np.column_stack([data[:, 0] ** i for i in range(thrust_to_signal_deg + 1)])
    b = data[:, 1]
    objective = cp.Minimize(cp.sum_squares(A @ thrust_to_signal_coeff - b))
    constraints = [
        A[0, :] @ thrust_to_signal_coeff == b[0],
        A[-1, :] @ thrust_to_signal_coeff == b[-1]
    ]
    prob = cp.Problem(objective, constraints)
    result = prob.solve(solver=cp.MOSEK)
    print(f'Signal to Thrust coefficients: {thrust_to_signal_coeff.value}')
    print(poly_string(thrust_to_signal_coeff.value))

    plt.scatter(data[:, 1], data[:, 0], c='r')
    signal = np.linspace(data[0, 1], data[-1, 1])
    A = np.column_stack([signal ** i for i in range(signal_to_thrust_deg + 1)])
    plt.plot(signal, A @ signal_to_thrust_coeff.value)
    plt.show()

    plt.scatter(data[:, 0], data[:, 1], c='r')
    thrust = np.linspace(data[0, 0], data[-1, 0])
    A = np.column_stack([thrust ** i for i in range(thrust_to_signal_deg + 1)])
    plt.plot(thrust, A @ thrust_to_signal_coeff.value)
    plt.show()

if __name__ == '__main__':
    main()
