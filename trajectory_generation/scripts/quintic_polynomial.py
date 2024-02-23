#!/usr/bin/env python3

import numpy as np
# import plotly.graph_objects as go 
# from plotly.subplots import make_subplots

def trajectory_cubic(x0, xf, x0_dot, xf_dot, x0_dot_dot, xf_dot_dot, time, interval):
        
    t0 = time[0]
    tf = time[-1]

    intervals = np.arange(t0, tf, interval)

    trajectories_matrix = np.empty((len(intervals), 9))

    for i in np.arange(0, 3, 1):

        coef = np.zeros((6,1)) #we are looking for this
        param = np.asarray([[x0[i]],[xf[i]],[x0_dot[i]],[xf_dot[i]],[x0_dot_dot[i]],[xf_dot_dot[i]]])
        matrix = np.asarray([[0,0,0,0,0,1],
            [tf**5,tf**4,tf**3,tf**2,tf,1],
            [0,0,0,0,1,0],
            [5*tf**4,4*tf**3,3*tf**2,2*tf,1,0],
            [0,0,0,2,0,0],
            [20*tf**3,12*tf**2,6*tf,2,0,0]])
        inverse_matrix = np.linalg.inv(matrix)
        coef = np.matmul(inverse_matrix, param)
        # print("Coefficients are:")
        # print(coef)

        for j in np.arange(0, len(intervals), 1):
            if i == 0:
                trajectories_matrix[j, 0] = coef[0]*(intervals[j]**5) + coef[1]*(intervals[j]**4) + coef[2]*(intervals[j]**3) + coef[3]*(intervals[j]**2) + coef[4]*(intervals[j]) + coef[5]
                trajectories_matrix[j, 1] = coef[0]*5*(intervals[j]**4) + coef[1]*4*(intervals[j]**3) + coef[2]*3*(intervals[j]**2) + coef[3]*2*(intervals[j]) + coef[4]
                trajectories_matrix[j, 2] = coef[0]*20*(intervals[j]**3) + coef[1]*12*(intervals[j]**2) + coef[2]*6*(intervals[j]) + coef[3]*2

            elif i == 1:
                trajectories_matrix[j, 3] = coef[0]*(intervals[j]**5) + coef[1]*(intervals[j]**4) + coef[2]*(intervals[j]**3) + coef[3]*(intervals[j]**2) + coef[4]*(intervals[j]) + coef[5]
                trajectories_matrix[j, 4] = coef[0]*5*(intervals[j]**4) + coef[1]*4*(intervals[j]**3) + coef[2]*3*(intervals[j]**2) + coef[3]*2*(intervals[j]) + coef[4]
                trajectories_matrix[j, 5] = coef[0]*20*(intervals[j]**3) + coef[1]*12*(intervals[j]**2) + coef[2]*6*(intervals[j]) + coef[3]*2
            
            elif i == 2:
                trajectories_matrix[j, 6] = coef[0]*(intervals[j]**5) + coef[1]*(intervals[j]**4) + coef[2]*(intervals[j]**3) + coef[3]*(intervals[j]**2) + coef[4]*(intervals[j]) + coef[5]
                trajectories_matrix[j, 7] = coef[0]*5*(intervals[j]**4) + coef[1]*4*(intervals[j]**3) + coef[2]*3*(intervals[j]**2) + coef[3]*2*(intervals[j]) + coef[4]
                trajectories_matrix[j, 8] = coef[0]*20*(intervals[j]**3) + coef[1]*12*(intervals[j]**2) + coef[2]*6*(intervals[j]) + coef[3]*2
    
    file_path = '/home/pi/trajectory.csv'

    np.savetxt(file_path, trajectories_matrix, delimiter=',')

    # fig = make_subplots(rows=3, cols=1, subplot_titles=['Position', 'Velocity', 'Acceleration'])

    # # Add traces to the first subplot
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,0], mode='lines', name='Position X'), row=1, col=1)
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,3], mode='lines', name='Position Y'), row=1, col=1)
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,6], mode='lines', name='Position Z'), row=1, col=1)

    # # Add traces to the second subplot
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,1], mode='lines', name='Velocity X'), row=2, col=1)
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,4], mode='lines', name='Velocity Y'), row=2, col=1)
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,7], mode='lines', name='Velocity Z'), row=2, col=1)

    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,2], mode='lines', name='Acceleration X'), row=3, col=1)
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,5], mode='lines', name='Acceleration Y'), row=3, col=1)
    # fig.add_trace(go.Scatter(x=intervals, y=trajectories_matrix[:,8], mode='lines', name='Acceleration Z'), row=3, col=1)

    # # Update layout for better appearance
    # fig.update_layout(title_text='Trajectory planning', showlegend=True)

    # # Show the plot
    # fig.show()

t = np.arange(0,4)

x0 = np.asarray([0,0,0])
x0_dot = np.asarray([0,0,0])
x0_dot_dot = np.asarray([0,0,0])

xf = np.asarray([0,0,1])
xf_dot = np.asarray([0,0,0])
xf_dot_dot = np.asarray([0,0,0])

trajectory_cubic(x0=x0,xf=xf, x0_dot=x0_dot, xf_dot=xf_dot, x0_dot_dot=x0_dot_dot, 
                 xf_dot_dot=xf_dot_dot, time=t, interval=0.01)
