#!/usr/bin/env python

import sys
import socket
import numpy as np
import time
import xml.etree.ElementTree as ET

def create_rsi_xml_rob(act_joint_pos, setpoint_joint_pos, ipoc):
    q = act_joint_pos
    qd = setpoint_joint_pos
    root = ET.Element('Rob', {'TYPE':'KUKA'})
    ET.SubElement(root, 'RIst', {'X':'0.0', 'Y':'0.0', 'Z':'0.0',
                                 'A':'0.0', 'B':'0.0', 'C':'0.0'})
    ET.SubElement(root, 'RSol', {'X':'0.0', 'Y':'0.0', 'Z':'0.0',
                                 'A':'0.0', 'B':'0.0', 'C':'0.0'})
    ET.SubElement(root, 'AIPos', {'A1':str(q[0]), 'A2':str(q[1]), 'A3':str(q[2]),
                                  'A4':str(q[3]), 'A5':str(q[4]), 'A6':str(q[5])})
    ET.SubElement(root, 'ASPos', {'A1':str(qd[0]), 'A2':str(qd[1]), 'A3':str(qd[2]),
                                  'A4':str(qd[3]), 'A5':str(qd[4]), 'A6':str(qd[5])})
    ET.SubElement(root, 'Delay', {'D':'0'})
    ET.SubElement(root, 'IPOC').text=str(ipoc)
    return ET.tostring(root)

def parse_rsi_xml_sen(data):
    root = ET.fromstring(data)
    AK = root.find('AK').attrib
    desired_joint_correction = np.array([AK['A1'], AK['A2'], AK['A3'],
                                         AK['A4'], AK['A5'], AK['A6']]).astype(np.float64)
    IPOC = root.find('IPOC').text
    return desired_joint_correction, int(IPOC)

# create dgram udp socket
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print 'Failed to create socket'
    sys.exit()

#host = 'localhost'
host = '192.168.1.67'
port = 49152
cycle_time = 0.004

act_joint_pos = np.array([0, -90, 90, 0, 90, 0]).astype(np.float64)
cmd_joint_pos = act_joint_pos.copy()
des_joint_correction_absolute = np.zeros(6)

ipoc = 0

while(True):
    try:
        msg = create_rsi_xml_rob(act_joint_pos, cmd_joint_pos, ipoc)
        s.sendto(msg, (host, port))
        recv_msg, addr = s.recvfrom(1024)
        t0 = time.time()
        des_joint_correction_absolute, ipoc_recv = parse_rsi_xml_sen(recv_msg)
        act_joint_pos = cmd_joint_pos + des_joint_correction_absolute
        ipoc += 1
        time.sleep(cycle_time / 2)
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
