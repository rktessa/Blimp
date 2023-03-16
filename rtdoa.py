
import math
import numpy as np

def TDoA(ts, dt):
    # Time definition
    '''t6_rx1 = float(ts[0,0]) * 15.65e-12
    t1_rx1 = float(ts[1,0]) * 15.65e-12
    t2_rx1 = float(ts[2,0]) * 15.65e-12
    t3_rx1 = float(ts[3,0]) * 15.65e-12
    t4_rx1 = float(ts[4,0]) * 15.65e-12
    t5_rx1 = float(ts[5,0]) * 15.65e-12

    t6_rx2 = float(ts[0,1]) * 15.65e-12
    t1_rx2 = float(ts[1,1]) * 15.65e-12
    t2_rx2 = float(ts[2,1]) * 15.65e-12
    t3_rx2 = float(ts[3,1]) * 15.65e-12
    t4_rx2 = float(ts[4,1]) * 15.65e-12
    t5_rx2 = float(ts[5,1]) * 15.65e-12 #double(1/(63.8976 * 100000000float

    t6_tx1 = float(ts[0,2]) * 15.65e-12
    t1_tx1 = float(ts[1,2]) * 15.65e-12
    t2_tx1 = float(ts[2,2]) * 15.65e-12
    t3_tx1 = float(ts[3,2]) * 15.65e-12
    t4_tx1 = float(ts[4,2]) * 15.65e-12
    t5_tx1 = float(ts[5,2]) * 15.65e-12

    t6_tx2 = float(ts[0,3]) * 15.65e-12
    t1_tx2 = float(ts[1,3]) * 15.65e-12
    t2_tx2 = float(ts[2,3]) * 15.65e-12
    t3_tx2 = float(ts[3,3]) * 15.65e-12
    t4_tx2 = float(ts[4,3]) * 15.65e-12
    t5_tx2 = float(ts[5,3]) * 15.65e-12
    '''
    
    t6_rx1 = float(ts[0]) * 15.65e-12
    t1_rx1 = float(ts[1]) * 15.65e-12
    t2_rx1 = float(ts[2]) * 15.65e-12
    t3_rx1 = float(ts[3]) * 15.65e-12
    t4_rx1 = float(ts[4]) * 15.65e-12
    t5_rx1 = float(ts[5]) * 15.65e-12

    t6_rx2 = float(ts[6]) * 15.65e-12
    t1_rx2 = float(ts[7]) * 15.65e-12
    t2_rx2 = float(ts[8]) * 15.65e-12
    t3_rx2 = float(ts[9]) * 15.65e-12
    t4_rx2 = float(ts[10]) * 15.65e-12
    t5_rx2 = float(ts[11]) * 15.65e-12 #double(1/(63.8976 * 100000000float

    t6_tx1 = float(ts[12]) * 15.65e-12
    t1_tx1 = float(ts[13]) * 15.65e-12
    t2_tx1 = float(ts[14]) * 15.65e-12
    t3_tx1 = float(ts[15]) * 15.65e-12
    t4_tx1 = float(ts[16]) * 15.65e-12
    t5_tx1 = float(ts[17]) * 15.65e-12

    t6_tx2 = float(ts[18]) * 15.65e-12
    t1_tx2 = float(ts[19]) * 15.65e-12
    t2_tx2 = float(ts[20]) * 15.65e-12
    t3_tx2 = float(ts[21]) * 15.65e-12
    t4_tx2 = float(ts[22]) * 15.65e-12
    t5_tx2 = float(ts[23]) * 15.65e-12
    
    # Embedded Lab system anchor position
    '''A_n1 = np.array([0.00, 7.19, 2.15])
    A_n2 = np.array([0.00, 3.62, 3.15])
    A_n3 = np.array([0.00, 0.00, 2.15])
    A_n4 = np.array([4.79, 1.85, 3.15])
    A_n5 = np.array([4.79, 5.45, 2.15])
    A_n6 = np.array([3.00, 9.35, 3.15])'''

    A_n1 = [0.00, 7.19, 2.15]
    A_n2 = [0.00, 3.62, 3.15]
    A_n3 = [0.00, 0.00, 2.15]
    A_n4 = [4.79, 1.85, 3.15]
    A_n5 = [4.79, 5.45, 2.15]
    A_n6 = [3.00, 9.35, 3.15]

    A_n = np.array((A_n6, A_n1, A_n2, A_n3, A_n4, A_n5))
    c = 299792458 # Speed of light
    n = len(A_n)

    
    #TOF_MA = np.sqrt(np.sum)

    # Real measurements
    toa_tx = np.array([[t6_tx1,t6_tx2],[t1_tx1, t1_tx2], [t2_tx1,t2_tx2], [t3_tx1,t3_tx2], [t4_tx1,t4_tx2], [t5_tx1,t5_tx2]])
    toa_rx = np.array([[t6_rx1,t6_rx2], [t1_rx1,t1_rx2], [t2_rx1,t2_rx2], [t3_rx1,t3_rx2], [t4_rx1,t4_rx2], [t5_rx1,t5_rx2]])

    #Drift tag
    dt_new = (toa_rx[:,1]- toa_rx[:,0]) /(toa_tx[:,1] - toa_tx[:,0])
    
    
    tmp_rx = np.zeros((len(toa_rx),2))
    tmp_rx[:][0] = toa_rx[:][0] - toa_rx[0][0] - (toa_tx[:][0] * dt -toa_tx[0][0]*dt)
    tmp_rx[:][1] = toa_rx[:][1] - toa_rx[0][1] - (toa_tx[:][1] * dt -toa_tx[0][1]*dt)
    
    ## TDoA
    #     tdoa = tmp_rx(:,2) - tmp_tx(:,2);
    tdoa = np.zeros((len(tmp_rx), 2))
    tdoa = tmp_rx[:, 1]
    tdoa = np.delete(tdoa, [0])

    D = c*tdoa
    
    #------Trilateration linear equations system-------------------
    A = np.array((A_n6[0] - A_n[1:n,0], A_n6[1]-A_n[1:n, 1], A_n6[2]-A_n[1:n, 2], D)) *2
    
    b = D**2 + np.linalg.norm(A_n6)**2 - np.sum(np.square(A_n[1:n, :]), axis=1)
    
   
    
    x_t0 = np.matmul(np.linalg.pinv(A), b.T)
    
    #print("x_t0 =", x_t0) 
    #print("A =", A)
    #print("pinvA = ", sp.linalg.pinv(A))
    #-----Non linear correction (Taylor Expansion)-----------------
    x_t_0 = np.array((x_t0[0], x_t0[1], x_t0[2]))
    f = np.zeros((n-1,1))
    del_f = np.zeros((n-1,3))
    ii = 1
    
    
    
    for ii in 1,n-1 :
        f[ii-1]= np.linalg.norm((x_t_0 - A_n[ii,:]), ord=2)-np.linalg.norm((x_t_0 - A_n[0,:]), ord = 2)
        del_f[ii-1,0] = (x_t_0[0] - A_n[ii,0])*np.linalg.norm((x_t_0 - A_n[ii,:]),ord=2)**-1 - (x_t_0[0]-A_n[0,0])*np.linalg.norm((x_t_0-A_n[0,:]), ord = 2)**-1
        del_f[ii-1,1] = (x_t_0[1] - A_n[ii,1])*np.linalg.norm((x_t_0 - A_n[ii,:]),ord=2)**-1 - (x_t_0[1]-A_n[0,1])*np.linalg.norm((x_t_0-A_n[0,:]), ord=2)**-1
        del_f[ii-1,2] = (x_t_0[2] - A_n[ii,2])*np.linalg.norm((x_t_0 - A_n[ii,:]), ord=2)**-1 - (x_t_0[2]-A_n[0,2])*np.linalg.norm((x_t_0-A_n[0,:]), ord=2)**-1 

    #print("res = ", (D-f.T) )
    apinv = np.linalg.pinv(del_f)
    abho = (D- f.T).T
    
    x_t = (np.matmul(np.linalg.pinv(del_f), (D- f.T).T)).T + x_t_0

    return x_t[0,0], x_t[0,1], x_t[0,2], dt_new