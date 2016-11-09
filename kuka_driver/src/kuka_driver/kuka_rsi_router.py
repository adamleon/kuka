#!/usr/bin/env python

# Copyright (c) 2014, Norwegian University of Science and Technology
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
# THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Lars Tingelstad
# Maintainer: Lars Tingelstad <lars.tingelstad@ntnu.no>


import socket
import threading
import time
import numpy as np
import struct
import xml.etree.ElementTree as et

class UDPServerRealTime(threading.Thread):

    def __init__(self,name, host, port, handshake=None):
        threading.Thread.__init__(self)
        self.daemon = True
        self.name = name

        self._host = host
        self._port = port

        self._handshake = handshake

        self._timeout = None
        self._timeout_count = 0
        self._is_timed_out = False
        self._max_timeout_count = None

        self._lock = threading.Lock()
        self._recv_data = None
        self._send_data = None

        self._remote_addr = None
        self.is_connected = False

        self._stop_flag = threading.Event()
        self._disconnect_client_flag = threading.Event()

        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.settimeout(self._timeout)
        self._socket.bind((self._host, self._port))

    def set_max_timeout_count(self, timeout_count):
        self._max_timeout_count = timeout_count

    def timeout(self):
        return self._timeout

    def set_timeout(self, timeout):
        self._timeout = timeout
        self._socket.settimeout(self._timeout)

    def receive(self):
        try:
            #self._lock.acquire()
            data, addr = self._socket.recvfrom(1024)
            self._recv_data = data
            #self._lock.release()

            ## Set connection if handshake mechanism is not used
            if self._handshake is None and not self.is_connected:
                self._remote_addr = addr
                print("{name}: Got connection from: {addr}".format(name=self.name, addr=self._remote_addr))
                self.is_connected = True
            self._timeout_count = 0
            return data

        except socket.timeout, e:
            if self._max_timeout_count is not None:
                self._timeout_count += 1
                print("{name}: Late package!".format(name=self.name))
                if self._timeout_count > self._max_timeout_count:
                    print("{name}: Maximum timeouts. Disconnecting client: {addr}".format(name=self.name, addr=self._remote_addr))
                    self._disconnect_client_flag.set()
            return None

    def send(self, data):
        #self._lock.acquire()
        self._send_data = data
        self._socket.sendto(self._send_data, self._remote_addr)
        #self._lock.release()

    def connect(self):
        ''' Create connection from external client '''
        if self._handshake is not None:
            if not self.is_connected:
                self._socket.settimeout(None)
                data, remote_addr = self._socket.recvfrom(1024)
                if data == self._handshake:
                    self._remote_addr = remote_addr
                    print("{name}: Got connection from: {addr}".format(name=self.name, addr=self._remote_addr))
                    self.is_connected = True
                else:
                    print("{name}: Could not accept connection from: {addr}".format(name=self.name, addr=remote_addr))
                    self._disconnect_client_flag.set()
            else:
                print("{name}: Can not create connection without handshake!".format(name=self.name))

        if self._timeout is not None:
            self._socket.settimeout(self._timeout)

    def stop(self):
        print("{name}: Stopping!".format(name=self.name))
        self._stop_flag.set()

    def disconnect(self):
        #print("{name}: Disconnecting!".format(name=self.name))
        self._disconnect_client_flag.set()

    def run(self):
        while not self._stop_flag.is_set():
            print("{name}: Waiting for connection!".format(name=self.name))
            if self._handshake is not None:
                self.connect()
            self._disconnect_client_flag.wait()
            print("{name}: Disconnecting client".format(name=self.name))
            self.is_connected = False
            self._remote_addr = None
            self._disconnect_client_flag.clear()
        self.join()

class KUKARSIRouter(object):

    def __init__(self):

        self._lock = threading.Lock()

        self._joint_correction = np.zeros(6).astype(np.float32)
        self._joint_setpoint_position_init = None

        #self._rsi_server = UDPServerRealTime('rsi server','localhost', 49152)
        self._rsi_server = UDPServerRealTime('rsi server','192.168.1.67', 49152)
        self._rsi_server.set_max_timeout_count(3)

        self._ext_control_server = UDPServerRealTime('ext control server', 'localhost', 10000, "RSI")
        self._ext_control_server.set_timeout(0.004)
        self._ext_control_server.set_max_timeout_count(3)

    def _parse_xml_from_robot(self, data):
        root = et.fromstring(data)
        # Cartesian actual position
        RIst = root.find('RIst').attrib
        cart_actual_pos = np.array([RIst['X'], RIst['Y'], RIst['Z'],
                                    RIst['A'], RIst['B'], RIst['C']], dtype=np.float64)
        # Cartesian setpoint position
        RSol = root.find('RSol').attrib
        cart_setpoint_pos = np.array([RSol['X'], RSol['Y'], RSol['Z'],
                                      RSol['A'], RSol['B'], RSol['C']], dtype=np.float64)
        # Axis actual
        AIPos = root.find('AIPos').attrib
        axis_actual_pos = np.array([AIPos['A1'], AIPos['A2'],AIPos['A3'],
                                    AIPos['A4'], AIPos['A5'],AIPos['A6']], dtype=np.float64)
        # Axis setpoint pos
        ASPos = root.find('ASPos').attrib
        axis_setpoint_pos = np.array([ASPos['A1'], ASPos['A2'],ASPos['A3'],
                                      ASPos['A4'], ASPos['A5'],ASPos['A6']], dtype=np.float64)
        # Number of late packages
        Delay = root.find('Delay').attrib
        n_late_packages = int(Delay['D'])
        # IPOC number
        IPOC = int(root.find('IPOC').text)
        return axis_actual_pos, axis_setpoint_pos, n_late_packages, IPOC

    def _create_xml_to_robot(self, desired_axis_corr, ipoc_cycle_num):
        dac = desired_axis_corr
        sen = et.Element('Sen', {'Type':'ImFree'})
        akorr = et.SubElement(sen, 'AK', {'A1':str(dac[0]),
                                          'A2':str(dac[1]),
                                          'A3':str(dac[2]),
                                          'A4':str(dac[3]),
                                          'A5':str(dac[4]),
                                          'A6':str(dac[5])})
        ipoc = et.SubElement(sen, 'IPOC').text = str(ipoc_cycle_num)
        return et.tostring(sen)

    def _create_joint_pos_packet(self, ipoc, axis_actual_pos):
        return struct.pack('Q6d', ipoc, *axis_actual_pos)

    def _parse_joint_pos_packet(self, packet):
        data = struct.unpack('Q6d', packet)
        ipoc = data[0]
        q_desired = np.array(data[1:], dtype=np.float64)
        return ipoc, q_desired

    def run(self):
        self._ext_control_server.start()
        self._rsi_server.start()
        #while not self._stop_flag.is_set():
        while True:
            ## Receive rsi packet from robot. This is a blocking call if no rsi
            ## is connected. The timeout is set to 4ms when the robot connects,
            ## and is reset to None when the robot disconnects.
            data = self._rsi_server.receive()
            if self._rsi_server.is_connected:
                ## Set timeout of receive for RSI client when robot connects
                if self._rsi_server.timeout() is None:
                    self._rsi_server.set_timeout(0.004)
                ## Only parse rsi packet if content is not None
                if data is not None:
                    ## Parse rsi packet xml document
                    q_actual, q_setpoint, late_packages, ipoc = self._parse_xml_from_robot(data)
                    if self._joint_setpoint_position_init is None:
                        self._joint_setpoint_position_init = q_setpoint
                    if self._ext_control_server.is_connected:
                        ipoc_out = ipoc
                        ## Create joint position packet to send to external control client
                        packet = self._create_joint_pos_packet(ipoc_out, q_actual)
                        ## Send send joint position packet to external control client
                        self._ext_control_server.send(packet)
                        ## Receive desired joint position packet
                        data = self._ext_control_server.receive()
                        if data is not None:
                            ## parse data from client
                            ipoc_in, q_desired = self._parse_joint_pos_packet(data)
                            print(q_desired)
                            ## check if the received ipoc timestamp is equal to
                            ## the received ipoc timestamp from the external
                            ## control client
                            if ipoc_in == ipoc_out:
                                ## The joint correction is equal to the desired joint
                                # position minus the current joint setpoint.
                                with self._lock:
                                    #self._joint_correction = q_desired - self._joint_setpoint_position_init
                                    self._joint_correction = q_desired - q_setpoint

                with self._lock:
                    data = self._create_xml_to_robot(self._joint_correction, ipoc)
                    print(data)
                self._rsi_server.send(data)
            else:
                print("RSI Router: No connection with robot. Disconnecting all external connections!")
                self._joint_setpoint_position_init = None
                self._joint_correction = np.zeros(6).astype(np.float32)
                self._ext_control_server.disconnect()
                self._rsi_server.set_timeout(None)

        self._ext_control_server.stop()
        self._rsi_server.stop;


if __name__ == '__main__':
    router = KUKARSIRouter()
    router.run()
