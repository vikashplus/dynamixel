from dynamixel_py import *
import time as t
import numpy as np


# Make pretty plots to show off your movements
def plot_paths(paths, filename):
    import matplotlib as mpl
    mpl.use('TkAgg')
    import matplotlib.pyplot as plt

    for i in range(len(paths)):
        plt.clf()
        ax = plt.subplot(3, 1, 1)
        plt.plot(paths[i]['qpos'], '-')
        ax.set_prop_cycle(None)
        plt.plot(paths[i]['ctrl'], '-', alpha=0.3, linewidth=5.0)
        plt.title(filename)
        plt.ylabel('qpos')
        # ax.set_ylim(-2, 2)
        
        ax = plt.subplot(3, 1, 2)
        plt.plot(paths[i]['qvel'], '-')
        ax.set_prop_cycle(None)
        plt.plot((paths[i]['qpos'][1:,:] - paths[i]['qpos'][:-1,:])*100,'--')
        plt.ylabel('qvel')
        # ax.set_ylim(-6, 6)

        ax = plt.subplot(3, 1, 3)
        plt.plot(paths[i]['ctrl'], '-', alpha=0.3, linewidth=5.0)
        plt.ylabel('ctrl')
        plt.xlabel('time')
        # ax.set_ylim(-2, 2)

        plt.tight_layout()
        fn = filename+'_path'+str(i)+'.png'
        plt.savefig(fn)
        print("path saved to " + fn)


def chirp(dy, dxl_ids, time_horizon):
    clk = []
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
        des_pos = [pos_mean + pos_scale*np.sin(4.0*np.pi*t_n)*np.cos(4.0*t_n)]
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




if __name__ == '__main__':
    
    print("============= dxl ==============")
    dxl_ids = [10]
    
    # Connect
    dy = dxl(dxl_ids)

    # Test update rate
    # test_update_rate(dy, dxl_ids, 1000)

    # Move all the joints and plot the trace
    trace = chirp(dy, dxl_ids, 5)
    plot_paths(trace, 'chirp')

    # Close
    dy.close(dxl_ids)
