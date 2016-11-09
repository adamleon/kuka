import socket
import struct
import time
import numpy as np


i = 0
w = 12
a = 0.05

t0 = None

act = []
cmd = []

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# send control word to rsi router for connection
sock.sendto("RSI", ('localhost', 10000))


last_correction = 0
q_start = None

is_stepped = False
print_step_time = False

for i in range(250):
    data, addr = sock.recvfrom(1024)
    act_time = time.time()
    if t0 is None:
        t0 = time.time()

    recv = struct.unpack('Q6d', data)
    ipoc = recv[0]
    q_actual = np.array(recv[1:], dtype=np.float64)
    print(q_actual)
    if q_start is None:
        q_start = q_actual.copy()

    act.append([act_time] + q_actual.tolist())
    q_desired = q_start.copy()
    ## step response test
    if i < 51:
        correction = 0.0
        t_step = time.time() - t0
    else:
        correction = 0.1

    if q_actual[4] != q_start[4]:
        if print_step_time is False:
            print(t_step)
            stepped_time = time.time() - t0
            print(stepped_time)
            print(stepped_time - t_step)
            print_step_time = True

    ### RESULTS FROM STEP TEST
    # Commanded step time: 0.200692892075
    # First received change time: 0.21674990654
    # Time from commanded until changed: 0.0160570144653
    ###

    ## tracking cosine
    #correction = np.rad2deg(a * (1 - np.cos(w * (time.time() - t0) )))
    q_desired[4] += correction

    send = struct.pack('Q6d', ipoc, *q_desired)
    sock.sendto(send, ('localhost', 10000))
    cmd.append([time.time()] + q_desired.tolist())
    last_correction = correction

#np.save('act.npy', np.array(act))
#np.save('cmd.npy', np.array(cmd))

#cmd=np.load('cmd.npy')
#act=np.load('act.npy')

#import matplotlib.pyplot as pp
#cmdt = cmd[:,0]
#actt = act[:,0]
#actp = act[:,5]
#cmdp = cmd[:,5]
##actp-=90
#pp.plot(cmdt-actt[0], cmdp)
#pp.plot(actt-actt[0], actp)

#pp.show()
