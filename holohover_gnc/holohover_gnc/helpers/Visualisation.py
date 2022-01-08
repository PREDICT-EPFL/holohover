import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt

dir = os.path.dirname(os.path.realpath(__file__))
path = dir + '/results/model_validation/'
X = np.genfromtxt(dir+'/results/model_validation/X.csv', delimiter=',')
U = np.genfromtxt(dir+'/results/model_validation/U.csv', delimiter=',')
F = np.genfromtxt(dir+'/results/model_validation/F.csv', delimiter=',')*10**3
t = np.genfromtxt(dir+'/results/model_validation/t.csv', delimiter=',')

print(F[-1,:])
def plotStates(X, t):
    #Plot joint positions
    fig, axs = plt.subplots(2, 3, sharex=True)
    fig.set_size_inches(20, 10)
    axs[0][0].plot(t,X[:,0], label='Yaw') 
    axs[0][0].set_xlabel('time [s]')
    axs[0][0].set_ylabel('angle [rad]')
    axs[0][0].legend()
    axs[0][1].plot(t,X[:,1], label='Yaw_d') 
    axs[0][1].set_xlabel('time [s]')
    axs[0][1].set_ylabel('angular velocity [rad/s]')
    axs[0][1].legend()
    axs[1][0].plot(t,X[:,2], label='x') 
    axs[1][0].set_xlabel('time [s]')
    axs[1][0].set_ylabel('position [m]')
    axs[1][0].legend()
    axs[1][1].plot(t,X[:,3], label='y') 
    axs[1][1].set_xlabel('time [s]')
    axs[1][1].set_ylabel('position [rad]')
    axs[1][1].legend()
    axs[0][2].plot(t,X[:,4], label='v_x') 
    axs[0][2].set_xlabel('time [s]')
    axs[0][2].set_ylabel('velocity [m/s]')
    axs[0][2].legend()
    axs[1][2].plot(t,X[:,5], label='v_y') 
    axs[1][2].set_xlabel('time [s]')
    axs[1][2].set_ylabel('velocity [m/s]')
    axs[1][2].legend()
    #plt.suptitle('State Evolution of the Hovercraft')
    plt.savefig(path+'State_Evolution.png')
    plt.show()

def plotMotorCommands(F,t):
    #Plot joint positions
    fig, axs = plt.subplots(2, 3, sharex=True)
    fig.set_size_inches(20, 10)
    axs[0][0].plot(t,F[:,0], label='Motor_A_1') 
    axs[0][0].set_xlabel('time [s]')
    axs[0][0].set_ylabel('thrust [mN]')
    axs[0][0].set_ylim([0,60])
    axs[0][0].legend()
    axs[0][1].plot(t,F[:,1], label='Motor_A_2') 
    axs[0][1].set_xlabel('time [s]')
    axs[0][1].set_ylabel('thrust [mN')
    axs[0][1].set_ylim([0,60])
    axs[0][1].legend()
    axs[1][0].plot(t,F[:,2], label='Motor_B_1') 
    axs[1][0].set_xlabel('time [s]')
    axs[1][0].set_ylabel('thrust [mN]')
    axs[1][0].set_ylim([0,60])
    axs[1][0].legend()
    axs[1][1].plot(t,F[:,3], label='Motor_B_2') 
    axs[1][1].set_xlabel('time [s]')
    axs[1][1].set_ylabel('thrust [mN]')
    axs[1][1].set_ylim([0,60])
    axs[1][1].legend()
    axs[0][2].plot(t,F[:,4], label='Motor_C_1') 
    axs[0][2].set_xlabel('time [s]')
    axs[0][2].set_ylabel('thrust [mN]')
    axs[0][2].set_ylim([0,60])
    axs[0][2].legend()
    axs[1][2].plot(t,F[:,5], label='Motor_C_2') 
    axs[1][2].set_xlabel('time [s]')
    axs[1][2].set_ylabel('thrust [mN]')
    axs[1][2].set_ylim([0,60])
    axs[1][2].legend()
    #plt.suptitle('Thrust forces of the Hovercraft')
    plt.savefig(path+'Thrust_Forces.png')
    plt.show() 

def plotAccelerations(U,t):
    fig, axs = plt.subplots(1, 3, sharex=True)
    fig.set_size_inches(20, 10)
    axs[0].plot(t,U[:,0], label='a_x') 
    axs[0].set_xlabel('time [s]')
    axs[0].set_ylabel('linear acceleration [m/s^2]')
    axs[0].legend()
    axs[1].plot(t,U[:,1], label='a_y') 
    axs[1].set_xlabel('time [s]')
    axs[1].set_ylabel('linear acceleration [m/s^2]')
    axs[1].legend()
    axs[2].plot(t,U[:,2], label='\gamma_dd') 
    axs[2].set_xlabel('time [s]')
    axs[2].set_ylabel('angular acceleration [rad/s^2]')
    axs[2].legend()
    #plt.suptitle('Acceleration Inputs of the Hovercraft')
    plt.savefig(path+'Acceleration_Input.png')
    plt.show()

plotAccelerations(U,t)
plotMotorCommands(F,t)
plotStates(X,t)
