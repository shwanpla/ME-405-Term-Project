"""
Lightweight 4th-order Runge–Kutta (RK4) ODE solver for MicroPython + ulab.

The solver integrates a system of ODEs of the form:

    x_dot, y = f(t, x)

where:
    - x is the state vector (as a column ulab/numpy array, shape (n_states, 1))
    - y is an output vector (any dimension), returned by the same function

This module is designed for:
    - Embedded control / estimation tasks
    - Use with ulab's subset of NumPy for speed and memory efficiency
"""

from ulab import numpy as np


def RK4_solver(fcn, x_0, tspan, tstep):
    """
    Integrate a system using a 4th-order Runge–Kutta method (RK4).

    The ODE function must have the signature:

        x_dot, y = fcn(t, x)

    where:
        - x_dot is the state derivative (same shape as x)
        - y is the output vector at time t

    :param fcn: ODE function handle f(t, x) -> (x_dot, y)
    :type fcn: callable
    :param x_0: Initial state vector (column array, shape (n_states, 1))
    :type x_0: ulab.numpy.ndarray
    :param tspan: Time span [t0, tf] for integration
    :type tspan: list | tuple
    :param tstep: Integration step size
    :type tstep: float
    :return: (tout, yout) where:
             - tout is a 1D array of time samples
             - yout is a 2D array of outputs, shape (len(tout), n_outputs)
    :rtype: tuple(ulab.numpy.ndarray, ulab.numpy.ndarray)
    """
    # Time vector
    num_steps = int(round((tspan[1] - tspan[0]) / tstep)) + 1
    tout = np.array([tspan[0] + i * tstep for i in range(num_steps)])

    # State storage
    n_states = x_0.shape[0]
    xout = np.zeros((len(tout) + 1, n_states))

    # Determine output dimension from a test call
    _, y_test = fcn(0, x_0)
    n_outputs = y_test.shape[0]

    # Output storage
    yout = np.zeros((len(tout), n_outputs))

    # Initialize state history with initial condition
    for i in range(n_states):
        xout[0, i] = x_0[i, 0]

    # RK4 integration loop
    for n in range(len(tout)):
        # Current state as column vector
        x = np.array([[xout[n, i]] for i in range(n_states)])
        t = tout[n]

        # k1
        k1, y = fcn(t, x)

        # k2
        k2, _ = fcn(t + 0.5 * tstep, x + 0.5 * k1 * tstep)

        # k3
        k3, _ = fcn(t + 0.5 * tstep, x + 0.5 * k2 * tstep)

        # k4
        k4, _ = fcn(t + tstep, x + k3 * tstep)

        # RK4 weighted average
        dx = (k1 + 2 * k2 + 2 * k3 + k4) * tstep / 6.0

        # Update state history
        for i in range(n_states):
            xout[n + 1, i] = xout[n, i] + dx[i, 0]

        # Store output at this time step
        for i in range(n_outputs):
            yout[n, i] = y[i, 0]

    return tout, yout
