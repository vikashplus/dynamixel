from dynamixel_py import *
import time as t
import numpy as np
import scipy.io as sio
    
global update_rate


# Make pretty plots to show off your movements
def plot_paths(paths, filename, qpos_lims=None, qvel_lims=None, ctrl_lims=None):
    import matplotlib as mpl
    mpl.use('TkAgg')
    import matplotlib.pyplot as plt

    for i in range(len(paths)):
        plt.clf()

        # time
        if('time' in paths[i].keys()):
            time = paths[i]['time']
        else:
            n = len(paths[i]['qpos'])
            time = np.linspace(0, n, n)/update_rate

        # positions
        ax = plt.subplot(3, 1, 1)
        plt.plot(time,paths[i]['qpos'], '-')
        ax.set_prop_cycle(None)
        plt.plot(time, paths[i]['ctrl'], '-', alpha=0.3, linewidth=5.0)
        plt.title(filename)
        plt.ylabel('qpos')
        if(qpos_lims):
            ax.set_ylim(qpos_lims[0], qpos_lims[1])
        
        # Velocities
        ax = plt.subplot(3, 1, 2)
        h0 = plt.plot(time,paths[i]['qvel'], '-')
        ax.set_prop_cycle(None)
        vel = (paths[i]['qpos'][1:,:] - paths[i]['qpos'][:-1,:])/(time[1:]-time[:-1]).reshape(-1,1)
        h1 = plt.plot(time[:-1], vel, '--', alpha=0.3)
        plt.ylabel('qvel')
        plt.legend((h0[0], h1[0]), ('qvel', 'fd(qpos)'))
        if(qvel_lims):
            ax.set_ylim(qvel_lims[0], qvel_lims[1])

        # controls
        ax = plt.subplot(3, 1, 3)
        plt.plot(time, paths[i]['ctrl'], '-', alpha=0.3, linewidth=5.0)
        plt.ylabel('ctrl')
        plt.xlabel('time')
        if(ctrl_lims):
            ax.set_ylim(ctrl_lims[0], ctrl_lims[1])

        # save plots
        plt.tight_layout()
        fn = filename+'_path'+str(i)+'.png'
        plt.savefig(fn)
        print("path saved to " + fn)


def chirp(dy, dxl_ids, frequency=2.0, time_horizon=5.0):
    clk =[]
    qpos=[]
    qvel=[]
    ctrl=[]

    pos_min = 0 
    pos_max = 2.0*np.pi
    pos_mean = (pos_max + pos_min)/2.0
    pos_scale = (pos_max - pos_min)/2.0
    
    print("Subjecting system to chirp signal");
    t_s = time.time()
    t_n = time.time() - t_s
    while(t_n < time_horizon):
        t_n = time.time() - t_s
        
        qp, qv = dy.get_pos_vel(dxl_ids)
        des_pos = [pos_mean + pos_scale*np.sin(frequency*2.0*np.pi*t_n)*np.cos(frequency*2.0*t_n)]*np.ones(len(dxl_ids))
        # print(des_pos)
        dy.set_des_pos(dxl_ids, des_pos)

        clk.append(t_n)
        qpos.append(qp)
        qvel.append(qv)
        ctrl.append(des_pos.copy())

    # Paths
    paths =[]
    path = dict(
        time=np.array(clk),
        qpos=np.array(qpos),
        qvel=np.array(qvel),
        ctrl=np.array(ctrl)
        )
    paths.append(path)

    return paths


def step(dy, dxl_ids, frequency=1.0, time_horizon=5.0):
    clk =[]
    qpos=[]
    qvel=[]
    ctrl=[]

    pos_min = 0 
    pos_max = 2.0*np.pi
    pos_mean = (pos_max + pos_min)/2.0
    pos_scale = (pos_max - pos_min)/2.0
    
    print("Subjecting system to step signal");
    t_s = time.time()
    t_n = time.time() - t_s
    while(t_n < time_horizon):
        t_n = time.time() - t_s
        
        qp, qv = dy.get_pos_vel(dxl_ids)
        des_pos = [pos_mean + .95*pos_scale*(2.*(int(frequency*2*t_n)%2) -1.)]*np.ones(len(dxl_ids))
        dy.set_des_pos(dxl_ids, des_pos)

        clk.append(t_n)
        qpos.append(qp)
        qvel.append(qv)
        ctrl.append(des_pos.copy())

    # Paths
    paths =[]
    path = dict(
        time=np.array(clk),
        qpos=np.array(qpos),
        qvel=np.array(qvel),
        ctrl=np.array(ctrl)
        )
    paths.append(path)

    return paths



# Test my update rate. I got good reflexes
def test_update_rate(dy, dxl_ids, cnt = 1000):
    print("Testing update rate of dxl")
    t_s = time.time()
    for i in range(cnt):
        dxl_present_position, dxl_present_velocity = dy.get_pos_vel(dxl_ids)
        dy.set_des_pos(dxl_ids, dxl_present_position)
    t_e = time.time()
    update_rate = cnt/(t_e-t_s)
    print("Update rate of dxl %3.2f hz (%1.4f s)" % (update_rate, 1.0/update_rate))
    return update_rate




if __name__ == '__main__':
    
    global update_rate
    update_rate = 444

    print("============= dxl ==============")
    # dxl_ids = [10, 11, 12]
    # dxl_ids = [20, 21, 22]
    # dxl_ids = [30, 31, 32]
    # dxl_ids = [10, 11, 12, 20, 21, 22, 30, 31, 32]
    dxl_ids = np.array([40])
    # Connect
    dy = dxl(dxl_ids)

    # dy.engage_motor(dxl_ids, False)

    while True:
        dxl_present_position, dxl_present_velocity = dy.get_pos_vel(dxl_ids)
        print(dxl_present_position)

    # Test update rate
    update_rate = test_update_rate(dy, dxl_ids, 1000)

    # Move all the joints and plot the trace
    trace = chirp(dy, dxl_ids, 1, 4)
    plot_paths(trace, 'chirp')
    sio.savemat('chirp.mat', {'trace':trace})
    trace = step(dy, dxl_ids, 1, 4)
    plot_paths(trace, 'step')
    sio.savemat('step.mat', {'trace':trace})

    # Close
    dy.close(dxl_ids)
