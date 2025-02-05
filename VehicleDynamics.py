import numpy as np

def get_kinematics(u,theta,sa, L = 4.9):
    x_dot = u*np.cos(theta)
    y_dot = u* np.sin(theta)
    theta_dot = (u/L) * np.tan(sa)

    return x_dot, y_dot, theta_dot

def integrate(x_init, y_init, u_init, theta_init, sa_init, dt = 0.05):
    x_dot, y_dot, theta_dot = get_kinematics(u_init,theta_init,sa_init)
    x_new = x_init + dt*x_dot
    y_new = y_init + dt * y_dot
    theta_new = theta_init + dt*theta_dot
