"""

Example oneD_dynamic_force_control.py

Second-order 1D dynamic model where the input is force (m * acceleration).

Author: GitHub Copilot (adapted)

"""

 

import numpy as np

import matplotlib.pyplot as plt

import sys
from pathlib import Path
mithril_dir = Path(__file__).resolve().parents[1]
sys.path.append(str(mithril_dir))

from mobotpy.models import Cart

 

# Simulation parameters

SIM_TIME = 10.0

T = 0.04

t = np.arange(0, SIM_TIME, T)

N = np.size(t)

 

def vehicle(x, v, u, T, m):

    """Discrete-time 1D dynamic vehicle model.

 

    States: position x [m], velocity v [m/s]

    Input: u = F = m * a [N]

    """

    a = u / m

    v_new = v + T * a

    x_new = x + T * v

    return x_new, v_new

 

if __name__ == "__main__":

    # Allocate arrays

    x = np.zeros(N)

    v = np.zeros(N)

    u = np.zeros(N)

 

    # Initial conditions

    x[0] = 1.0

    v[0] = 0.0

 

    # System mass [kg]

    m = 1.0

 

    # Desired position [m]

    x_d = 4.0

 

    # Controller gains (tune as needed)

    k_P = 2.0

    k_D = 0.8

 

    # Controller gains (tune as needed)

    # k_P = 1

    # k_D = 3

 

    # Run simulation

    for k in range(1, N):

        x[k], v[k] = vehicle(x[k - 1], v[k - 1], u[k - 1], T, m)

        # PD on position/velocity to compute commanded acceleration

        a_cmd = k_P * (x_d - x[k]) - k_D * v[k]

        # Input is force = m * acceleration

        u[k] = m * a_cmd

 

    # Plot results

    plt.rc("text", usetex=False)

    # plt.rc("savefig", format="pdf")

    # plt.rc("savefig", bbox="tight")

 

    fig = plt.figure(1)

    ax1 = plt.subplot(311)

    plt.plot(t, x, "C0")

    plt.grid(color="0.95")

    plt.ylabel(r"$x$ [m]")

    plt.setp(ax1, xticklabels=[])

 

    ax2 = plt.subplot(312, sharex=ax1)

    plt.plot(t, v, "C1")

    plt.grid(color="0.95")

    plt.ylabel(r"$v$ [m/s]")

    plt.setp(ax2, xticklabels=[])

 

    ax3 = plt.subplot(313, sharex=ax1)

    plt.step(t, u, "C2", where="post")

    plt.grid(color="0.95")

    plt.ylabel(r"$u$ [N]")

    plt.xlabel(r"$t$ [s]")

 

    # plt.savefig("./oneD_dynamic_force_control_fig1.pdf")

 

    # Animation (uses only position)

    LENGTH = 1.0

    vehicle_anim = Cart(LENGTH)

    ani = vehicle_anim.animate(x, T)

 

    plt.show()