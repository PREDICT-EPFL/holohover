import numpy as np
import pandas as pd
import os
import matplotlib.pyplot as plt


dir = '/home/karim/holohover_ws'
'''
path = dir + '/results/model_validation/'
X = np.genfromtxt(dir+'/results/model_validation/X.csv', delimiter=',')
U = np.genfromtxt(dir+'/results/model_validation/U.csv', delimiter=',')
F = np.genfromtxt(dir+'/results/model_validation/F.csv', delimiter=',')*10**3
t = np.genfromtxt(dir+'/results/model_validation/t.csv', delimiter=',')
'''

def readResults(path):
    X = np.genfromtxt(path + 'simulator.csv', delimiter=',')
    F = np.genfromtxt(path + 'motor.csv', delimiter=',')
    X_hat = np.genfromtxt(path + 'estimator.csv', delimiter=',')
    Cam = np.genfromtxt(path + 'camera.csv', delimiter=',')
    IMU = np.genfromtxt(path + 'measurement.csv', delimiter=',')
    return X,F,X_hat,Cam,IMU

def cleanResults(path):
    X,F,X_hat,Cam,IMU = readResults(path)
    elements = [X,F,X_hat,IMU]
    min = 10**16
    for i in elements:
        if i.shape[0]<min:
            if i==Cam:
                min = int(i.shape[0]/30*200)
            else:
                min = i.shape[0]
    min_camera = int(min/200*30)
    X = X[-min:,:]
    X_hat = X_hat[-min:,:]
    F = F[-min:,:]
    IMU = IMU[-min:,:]
    Cam = Cam[-min_camera:,:]
    t = np.arange(0,(1/200)*min,(1/200))
    t_cam = np.arange(0,(1/30)*min_camera,(1/30))
    return X,F,X_hat,Cam,IMU,t,t_cam

def plotEstimator(X,X_hat,Cam,IMU,t,t_cam, idx):
    fig, axs = plt.subplots(1, 3, sharex=True)
    fig.set_size_inches(20, 10)
    idx_cam = int(idx/200*30)
    idx_offset = 96-112
    axs[0].plot(t[:idx],X[:idx,4], label='x_true') 
    axs[0].plot(t[:idx],X_hat[:idx,4], label='x_estimated') 
    axs[0].plot(t_cam[-idx_offset:idx_cam],Cam[:idx_cam+idx_offset,0], label='x_measured') 
    axs[0].set_xlabel('time [s]')
    axs[0].set_ylabel('position [m]')
    axs[0].legend()
    axs[1].plot(t[:idx],X[:idx,5], label='y_true') 
    axs[1].plot(t[:idx],X_hat[:idx,5], label='y_estimated') 
    axs[1].plot(t_cam[-idx_offset:idx_cam],Cam[:idx_cam+idx_offset,1], label='y_measured') 
    axs[1].set_xlabel('time [s]')
    axs[1].set_ylabel('position [m]')
    axs[1].legend()
    axs[2].plot(t[:idx],X[:idx,1], label='yaw_d_true') 
    axs[2].plot(t[:idx],X_hat[:idx,1], label='yaw_d_estimated') 
    axs[2].plot(t[:idx],IMU[:idx,8], label='yaw_d_measured') 
    axs[2].set_xlabel('time [s]')
    axs[2].set_ylabel('angular rate [rad/s]')
    axs[2].legend()

    #plt.suptitle('Acceleration Inputs of the Hovercraft')
    plt.savefig(path+'Estimator.png')
    plt.show()

def plotTracking(X_hat, t, t_cam, idx):
    length = X_hat.shape[0]
    ref_vx=0
    ref_vy=0
    ref_yawd =0
    Ref_vx = np.ones(shape=(X_hat.shape[0],1))*ref_vx
    Ref_vy = np.ones(shape=(X_hat.shape[0],1))*ref_vy
    Ref_yawd = np.ones(shape=(X_hat.shape[0],1))*ref_yawd
    fig, axs = plt.subplots(1, 3, sharex=True)
    fig.set_size_inches(20, 10)
    axs[0].plot(t[:idx],X_hat[:idx,2], label='vx_estimated')
    axs[0].plot(t[:idx],Ref_vx[:idx], label='vx_reference')    
    axs[0].set_xlabel('time [s]')
    axs[0].set_ylabel('linear velocity [m]')
    axs[0].legend()
    axs[1].plot(t[:idx],X_hat[:idx,3], label='vy_estimated') 
    axs[1].plot(t[:idx],Ref_vy[:idx], label='vy_reference')  
    axs[1].set_xlabel('time [s]')
    axs[1].set_ylabel('linear velocity [m]')
    axs[1].legend()
    axs[2].plot(t[:idx],X_hat[:idx,1], label='yaw_d_estimated')
    axs[2].plot(t[:idx],Ref_yawd[:idx], label='yaw_d_reference')    
    axs[2].set_xlabel('time [s]')
    axs[2].set_ylabel('angular rate [rad/s]')
    axs[2].legend()
    plt.savefig(path+'Tracking.png')
    plt.show()

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
    axs[1][0].plot(t,X[:,4], label='x') 
    axs[1][0].set_xlabel('time [s]')
    axs[1][0].set_ylabel('position [m]')
    axs[1][0].legend()
    axs[1][1].plot(t,X[:,5], label='y') 
    axs[1][1].set_xlabel('time [s]')
    axs[1][1].set_ylabel('position [rad]')
    axs[1][1].legend()
    axs[0][2].plot(t,X[:,2], label='v_x') 
    axs[0][2].set_xlabel('time [s]')
    axs[0][2].set_ylabel('velocity [m/s]')
    axs[0][2].legend()
    axs[1][2].plot(t,X[:,3], label='v_y') 
    axs[1][2].set_xlabel('time [s]')
    axs[1][2].set_ylabel('velocity [m/s]')
    axs[1][2].legend()
    #plt.suptitle('State Evolution of the Hovercraft')
    plt.savefig(path+'State_Evolution.png')
    plt.show()

def plotMotorCommands(F,t,idx): 
    #Plot joint positions
    fig, axs = plt.subplots(2, 3, sharex=True)
    fig.set_size_inches(20, 10)
    axs[0][0].plot(t[:idx],F[:idx,0], label='Motor_A_1') 
    axs[0][0].set_xlabel('time [s]')
    axs[0][0].set_ylabel('signal')
    #axs[0][0].set_ylim([0,1.2])
    axs[0][0].legend()
    axs[0][1].plot(t[:idx],F[:idx,1], label='Motor_A_2') 
    axs[0][1].set_xlabel('time [s]')
    axs[0][1].set_ylabel('thrust [mN')
    #axs[0][1].set_ylim([0,1.2])
    axs[0][1].legend()
    axs[1][0].plot(t[:idx],F[:idx,2], label='Motor_B_1') 
    axs[1][0].set_xlabel('time [s]')
    axs[1][0].set_ylabel('signal')
    #axs[1][0].set_ylim([0,1.2])
    axs[1][0].legend()
    axs[1][1].plot(t[:idx],F[:idx,3], label='Motor_B_2') 
    axs[1][1].set_xlabel('time [s]')
    axs[1][1].set_ylabel('signal')
    #axs[1][1].set_ylim([0,1.2])
    axs[1][1].legend()
    axs[0][2].plot(t[:idx],F[:idx,4], label='Motor_C_1') 
    axs[0][2].set_xlabel('time [s]')
    axs[0][2].set_ylabel('signal')
    #axs[0][2].set_ylim([0,1.2])
    axs[0][2].legend()
    axs[1][2].plot(t[:idx],F[:idx,5], label='Motor_C_2') 
    axs[1][2].set_xlabel('time [s]')
    axs[1][2].set_ylabel('signal')
    #axs[1][2].set_ylim([0,1.2])
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


path = dir + '/results/model_validation/'
X = np.genfromtxt(dir+'/results/model_validation/X.csv', delimiter=',')
U = np.genfromtxt(dir+'/results/model_validation/U.csv', delimiter=',')
F = np.genfromtxt(dir+'/results/model_validation/F.csv', delimiter=',')*10**3
t = np.genfromtxt(dir+'/results/model_validation/t.csv', delimiter=',')
#X,F,X_hat,Cam,IMU, t, t_cam = cleanResults(path)
#plotTracking(X_hat, t, t_cam, idx=2000)
#plotMotorCommands(F,t,idx=2000)
plotStates(X,t)