# -*- coding: utf-8 -*-
"""
Copyright (C) Mon Aug 22 11:12:40 2016  Jianshan Zhou
Contact: zhoujianshan@buaa.edu.cn	jianshanzhou@foxmail.com
Website: <https://github.com/JianshanZhou>

This program is free software: you can redistribute
 it and/or modify it under the terms of
 the GNU General Public License as published
 by the Free Software Foundation,
 either version 3 of the License,
 or (at your option) any later version.
 
This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY;
 without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE.
 See the GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with this program.
 If not, see <http://www.gnu.org/licenses/>.
 
This module defines a basic vehicle class.
"""
import numpy as np

from communication_terminal import Communication_ternimal


class Mobility(object):
    """
    Mobility class defines the dynamics of single vehicles in the net.
    It is worth pointing out that this model simply considers one-dimensional
    vehicle mobility scenarios.
    """

    def __init__(
        self,
        v0=30.0,  # desired speed in m/s
        T=1.5,  # safe time headway in s
        a=1.0,  # maximum acceleration in m/s^2
        b=3.0,  # desired deceleration in m/s^2
        delta=4.0,  # acceleration exponent
        s0=2,  # minimum distance in m
        l0=5.0,
    ):  # the length of each vehicle in m

        self.v0 = v0
        self.T = T
        self.a = a
        self.b = b
        self.delta = delta
        self.s0 = s0
        self.l0 = l0

    def car_following_IDM(self, front_vehicle, rear_vehicle):
        """
        Adopt the Intelligent Driver Model (IDM) for car following dynamics.
        """
        if isinstance(front_vehicle, Vehicle) and isinstance(rear_vehicle, Vehicle):

            x1 = front_vehicle.position[0]
            x2 = rear_vehicle.position[0]
            v1 = front_vehicle.speed[0]
            v2 = rear_vehicle.speed[0]

            net_distance = np.abs(x1 - x2) - self.l0

            if rear_vehicle.direct_flag == 1:
                dv = v2 - v1
                s = self.s0 + v2 * self.T + (v2 * dv) / (2 * np.sqrt(self.a * self.b))
                component1 = (v2 / self.v0) ** self.delta
                component2 = (s / net_distance) ** 2
                rear_vehicle.acceleration[0] = self.a * (1.0 - component1 - component2)
            else:
                v1 = np.abs(v1)
                v2 = np.abs(v2)
                dv = v2 - v1
                s = self.s0 + v2 * self.T + (v2 * dv) / (2 * np.sqrt(self.a * self.b))
                component1 = (v2 / self.v0) ** self.delta
                component2 = (s / net_distance) ** 2
                rear_vehicle.acceleration[0] = (-1.0) * (
                    self.a * (1.0 - component1 - component2)
                )
        else:
            rear_vehicle.acceleration[0] = 0.0


class Vehicle(object):
    """
    Vehicle class:
    the attributes consists of ID, direct_flag, lane_ID,
     position array([x, y]), and some kinematics parameters
     including speed array([vx, vy]) and acceleration array([ax, ay]).
    """

    def __init__(
        self,
        ID=0,  # ID of the vehicle, an integer
        direct_flag=1,  # an integer indicates the moving direction
        lane_ID=0,  # an integer indicates the lane number
        position=np.array([0.0, 0.0]),  # position vecotr in m
        speed=np.array([0.0, 0.0]),  # speed vector in m/s
        acceleration=np.array([0.0, 0.0]),
    ):  # acceleration in m/s^2
        self.ID = ID
        self.direct_flag = direct_flag
        self.lane_ID = lane_ID
        self.position = position
        self.speed = speed
        self.acceleration = acceleration

    def setup_communication_terminal(self, issource=False):
        self.communication_terminal = Communication_ternimal(self.ID, issource)

    def update_acceleration(self, mobility, front_vehicle=None):
        mobility.car_following_IDM(front_vehicle, self)

    def update_speed(self, dt):
        self.speed = self.speed + dt * self.acceleration

    def update_position(self, dt):
        self.position = self.position + self.speed * dt
