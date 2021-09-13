# -*- coding: utf-8 -*-
"""
Copyright (C) Mon Aug 22 16:55:50 2016  Jianshan Zhou
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
 
This module defines some basic parameters that will be used for setting up the
 VANET simulation scenarios.
"""

from scipy.stats import norm, expon, lognorm, fisk
import numpy as np
from vehicle import Vehicle, Mobility


class Speed_headway_random(object):
    def __init__(self, scenario_flag = "Freeway_Free"):
        """
        Totally five scenarios are supported here:
        Freeway_Night, Freeway_Free, Freeway_Rush;
        Urban_Peak, Urban_Nonpeak.
        The PDFs of the vehicle speed and the inter-vehicle space are adapted 
         from existing references.
        """
        if scenario_flag == "Freeway_Night":
            self.headway_random = expon(0.0, 1.0/256.41)
            meanSpeed = 30.93 #m/s
            stdSpeed = 1.2 #m/s
        elif scenario_flag == "Freeway_Free":
            self.headway_random = lognorm(0.75, 0.0, np.exp(3.4))
            meanSpeed = 29.15 #m/s
            stdSpeed = 1.5 #m/s
        elif scenario_flag == "Freeway_Rush":
            self.headway_random = lognorm(0.5, 0.0, np.exp(2.5))
            meanSpeed = 10.73 #m/s
            stdSpeed = 2.0 #m/s
        elif scenario_flag == "Urban_Peak":
            scale = 1.096
            c = 0.314
            loc = 0.0
            self.headway_random = fisk(c, loc, scale)
            meanSpeed = 6.083 #m/s
            stdSpeed = 1.2 #m/s
        elif scenario_flag == "Urban_Nonpeak":
            self.headway_random = lognorm(0.618, 0.0, np.exp(0.685))
            meanSpeed = 12.86 #m/s
            stdSpeed = 1.5 #m/s
        else:
            raise
        
        self.speed_random = norm(meanSpeed, stdSpeed)

        
class Scenario(object):
    def __init__(self,mobility,
                 road_length = 5.0*10**3, # the length of the road in meter
                 two_direction = True, # bool-type flag indicates the directions
                 lane_width = 3.0, # lane width
                 lane_num_per_direct = 2, # the number of lanes per direction
                 vehicle_length = 5.0, # length of a vehicle in meter
                 onehop_delay = 10.0*10**(-3), # the delay in one-hop communication in second
                 total_sim_time = 120.0, # total simulation time in second
                 scenario_flag = "Freeway_Free",
                 commR = 100.0):   
        
        self.ROAD_LENGTH = road_length
        self.TWO_DIRECTION = two_direction
        self.LANE_WIDTH = lane_width
        self.LANE_NUM_PER_DIRECT = lane_num_per_direct
        self.VEHICLE_LENGTH = vehicle_length
        self.ONEHOP_DELAY = onehop_delay
        self.TOTAL_SIM_TIME = total_sim_time
        self.speed_headway_random = Speed_headway_random(scenario_flag)
        self.COMMUNICATION_RANGE = commR
        self.right_vehicle_count = np.zeros((self.LANE_NUM_PER_DIRECT,))
        self.left_vehicle_count = np.zeros((self.LANE_NUM_PER_DIRECT,))
        
        self.mobility = mobility
        self.time_elapsed = 0.0
        self.propagation_distance = 0.0
        self.infected_ratio = 0.0
        self.current_message_carrier = []        
        
        self.right_arrival_dt = []
        self.left_arrival_dt = []
        for laneInd in range(self.LANE_NUM_PER_DIRECT):
            nextDt = (self.VEHICLE_LENGTH + \
            self.speed_headway_random.headway_random.rvs())/(self.speed_headway_random.speed_random.rvs())
            self.right_arrival_dt.append(nextDt)
            nextDt1 = (self.VEHICLE_LENGTH + \
            self.speed_headway_random.headway_random.rvs())/(self.speed_headway_random.speed_random.rvs())
            self.left_arrival_dt.append(nextDt1)
            
    def setup_vehicle_pools(self):
        
        self.left_pool = []
        self.right_pool = []
        
        vehicle_id = 0
        direct_flag = 1
        lane_ID = 0
        acceleration = np.array([0.0,0.0])
        speed = np.array([0.0,0.0])
        position = np.array([0.0,0.0])
        position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
        lane_ID*self.LANE_WIDTH)
        speed[0] = self.speed_headway_random.speed_random.rvs()
        vehicle = Vehicle(vehicle_id,direct_flag,lane_ID,
                         position,speed,acceleration)
        vehicle.setup_communication_terminal(True)# the source
        vehicle.communication_terminal.receive([vehicle_id])
        self.current_message_carrier.append(vehicle)
        self.propagation_distance = vehicle.position[0]
        # the right pool
        lane_pool = []        
        
        while position[0] < (self.ROAD_LENGTH - self.VEHICLE_LENGTH):
            lane_pool.append(vehicle)
            self.right_vehicle_count[lane_ID] += 1
            vehicle_id += 1
            acceleration = np.array([0.0,0.0])
            speed = np.array([0.0,0.0])
            speed[0] = self.speed_headway_random.speed_random.rvs()
            position = np.array([position[0],0.0])
            position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
            lane_ID*self.LANE_WIDTH)
            position[0] = position[0] + self.VEHICLE_LENGTH + self.speed_headway_random.headway_random.rvs()
            vehicle = Vehicle(vehicle_id,direct_flag,lane_ID,
                             position,speed,acceleration)
            vehicle.setup_communication_terminal(False)
            
        self.right_pool.append(lane_pool)
        
        for lane_ID in range(self.LANE_NUM_PER_DIRECT - 1):
            
            vehicle_id = 0
            acceleration = np.array([0.0,0.0])
            speed = np.array([0.0,0.0])
            position = np.array([0.0,0.0])
            position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
            (lane_ID + 1)*self.LANE_WIDTH)
            speed[0] = self.speed_headway_random.speed_random.rvs()
            vehicle = Vehicle(vehicle_id,direct_flag,(lane_ID + 1),
                 position,speed,acceleration)
            vehicle.setup_communication_terminal(False)

            lane_pool = []        
            
            while position[0] < (self.ROAD_LENGTH - self.VEHICLE_LENGTH):
                lane_pool.append(vehicle)
                self.right_vehicle_count[1+lane_ID] += 1
                vehicle_id += 1
                acceleration = np.array([0.0,0.0])
                speed = np.array([0.0,0.0])
                speed[0] = self.speed_headway_random.speed_random.rvs()
                position = np.array([position[0],0.0])
                position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
                (lane_ID+1)*self.LANE_WIDTH)
                position[0] = position[0] + self.VEHICLE_LENGTH + self.speed_headway_random.headway_random.rvs()
                vehicle = Vehicle(vehicle_id,direct_flag,lane_ID,
                                 position,speed,acceleration)
                vehicle.setup_communication_terminal(False)
                
            self.right_pool.append(lane_pool)
        print("successfully set up the right vehicle pool!")
        
        # the left pool
        direct_flag = 2
        for lane_ID in range(self.LANE_NUM_PER_DIRECT):
            
            vehicle_id = 0
            acceleration = np.array([0.0,0.0])
            speed = np.array([0.0,0.0])
            position = np.array([0.0,0.0])
            position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
            (lane_ID)*self.LANE_WIDTH)
            position[0] = self.ROAD_LENGTH - position[0]
            speed[0] = (-1)*self.speed_headway_random.speed_random.rvs()
            vehicle = Vehicle(vehicle_id,direct_flag,(lane_ID + 1),
                 position,speed,acceleration)
            vehicle.setup_communication_terminal(False)
            
            lane_pool = []        
            
            while position[0] > (self.VEHICLE_LENGTH):
                lane_pool.append(vehicle)
                self.left_vehicle_count[lane_ID] += 1
                vehicle_id += 1
                acceleration = np.array([0.0,0.0])
                speed = np.array([0.0,0.0])
                speed[0] = (-1)*self.speed_headway_random.speed_random.rvs()
                position = np.array([position[0],0.0])
                position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
                (lane_ID)*self.LANE_WIDTH)
                position[0] = position[0] + (-1)*(self.VEHICLE_LENGTH + self.speed_headway_random.headway_random.rvs())
                vehicle = Vehicle(vehicle_id,direct_flag,lane_ID,
                                 position,speed,acceleration)
                vehicle.setup_communication_terminal(False)
                
            self.left_pool.append(lane_pool)

        print("successfully set up the left vehicle pool!")
        self.infected_ratio = 1.0*len(self.current_message_carrier)/(1.0*sum([len(self.right_pool[lane_ID]) for lane_ID in range(self.LANE_NUM_PER_DIRECT)]) \
        +1.0*sum([len(self.left_pool[lane_ID]) for lane_ID in range(self.LANE_NUM_PER_DIRECT)]))


    def input_vehicle(self, direct_flag, lane_ID):
        
        if direct_flag == 1:
            self.right_vehicle_count[lane_ID] += 1
            
            vehicle_id = self.right_vehicle_count[lane_ID] - 1
            acceleration = np.array([0.0,0.0])
            speed = np.array([0.0,0.0])            
            speed[0] = self.speed_headway_random.speed_random.rvs()
            position = np.array([0.0,0.0])
            position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
                (lane_ID)*self.LANE_WIDTH)
            vehicle = Vehicle(vehicle_id,direct_flag,lane_ID,
                 position,speed,acceleration)
            vehicle.setup_communication_terminal(False)
            self.right_pool[lane_ID].insert(0,vehicle)
            
            #update the arrival dt
            self.right_arrival_dt[lane_ID] = (self.VEHICLE_LENGTH + \
            self.speed_headway_random.headway_random.rvs())/(self.speed_headway_random.speed_random.rvs())
            
        elif direct_flag == 2:
            self.left_vehicle_count[lane_ID] += 1
            
            vehicle_id = self.left_vehicle_count[lane_ID] - 1
            acceleration = np.array([0.0,0.0])
            speed = np.array([0.0,0.0])            
            speed[0] = (-1)*(self.speed_headway_random.speed_random.rvs())
            position = np.array([0.0,0.0])
            position[0] = self.ROAD_LENGTH - position[0]
            position[1] = ((-1)**(direct_flag))*(self.LANE_WIDTH*0.5 + \
                (lane_ID)*self.LANE_WIDTH)
            vehicle = Vehicle(vehicle_id,direct_flag,lane_ID,
                 position,speed,acceleration)
            vehicle.setup_communication_terminal(False)
            self.left_pool[lane_ID].insert(0,vehicle)
            
            # update the arrival dt
            self.left_arrival_dt[lane_ID] = (self.VEHICLE_LENGTH + \
            self.speed_headway_random.headway_random.rvs())/(self.speed_headway_random.speed_random.rvs())
            
    def update_position(self):
        
        # update the positions of all vehicles based on the mobility model
        for lane_ID in range(self.LANE_NUM_PER_DIRECT):
            
            #update right pool
            for ind in np.arange(-1,-1-len(self.right_pool[lane_ID]),-1):
                if ind == -1:
                    front_vehicle = -1
                else:
                    front_vehicle = self.right_pool[lane_ID][ind+1]
                
                self.right_pool[lane_ID][ind].update_acceleration(self.mobility, front_vehicle)
                self.right_pool[lane_ID][ind].update_speed(self.ONEHOP_DELAY)
                self.right_pool[lane_ID][ind].update_position(self.ONEHOP_DELAY)

            for vehicle in self.right_pool[lane_ID]:
                if vehicle.position[0] > (self.ROAD_LENGTH-self.VEHICLE_LENGTH):  
                    self.right_pool[lane_ID].remove(vehicle)
                    if vehicle in self.current_message_carrier:
                        self.current_message_carrier.remove(vehicle)
                    
            #update left pool
            for ind in np.arange(-1,-1-len(self.left_pool[lane_ID]),-1):
                if ind == -1:
                    front_vehicle = -1
                else:
                    front_vehicle = self.left_pool[lane_ID][ind+1]
                
                self.left_pool[lane_ID][ind].update_acceleration(self.mobility, front_vehicle)
                self.left_pool[lane_ID][ind].update_speed(self.ONEHOP_DELAY)
                self.left_pool[lane_ID][ind].update_position(self.ONEHOP_DELAY)

            for vehicle in self.left_pool[lane_ID]:
                if vehicle.position[0] < (self.VEHICLE_LENGTH):  
                    self.left_pool[lane_ID].remove(vehicle)
                    if vehicle in self.current_message_carrier:
                        self.current_message_carrier.remove(vehicle)

    def communication(self):
        new_message_carrier = []
        for message_carrier in self.current_message_carrier:
            for lane_ID in range(self.LANE_NUM_PER_DIRECT):
                # the right pool situation
                for vehicle in self.right_pool[lane_ID]:
                    if not vehicle.communication_terminal.received_flag:
                        if np.linalg.norm(message_carrier.position - vehicle.position) <=self.COMMUNICATION_RANGE:
                            message_carrier.communication_terminal.send(vehicle)
                            if vehicle.position[0] >= self.propagation_distance:
                                self.propagation_distance = vehicle.position[0]
                            #print "send a message from V-%d to V-%d"%(message_carrier.ID,vehicle.ID)
                            new_message_carrier.append(vehicle)
                # the left pool situation
                for vehicle in self.left_pool[lane_ID]:
                    if not vehicle.communication_terminal.received_flag:
                        if np.linalg.norm(message_carrier.position - vehicle.position) <= self.COMMUNICATION_RANGE:
                            message_carrier.communication_terminal.send(vehicle)
                            if vehicle.position[0] >= self.propagation_distance:
                                self.propagation_distance = vehicle.position[0]
                            #print "send a message from V-%d to V-%d"%(message_carrier.ID,vehicle.ID)
                            new_message_carrier.append(vehicle)
                        
        # update the message carrier pool
        self.current_message_carrier.extend(new_message_carrier)
        if self.infected_ratio < 1.0:
            self.infected_ratio = 1.0*len(self.current_message_carrier)/(1.0*sum([len(self.right_pool[lane_ID]) for lane_ID in range(self.LANE_NUM_PER_DIRECT)]) \
            +1.0*sum([len(self.left_pool[lane_ID]) for lane_ID in range(self.LANE_NUM_PER_DIRECT)]))
        else:
            self.infected_ratio = 1.0
        self.time_elapsed += self.ONEHOP_DELAY
        
        
#%% testing
def test1():
    mobility = Mobility()
    scenario = Scenario(mobility)
    scenario.setup_vehicle_pools()
    
def test2():
    # simulation demo
    # set up the scenario
    scenario = Scenario(Mobility())
    scenario.setup_vehicle_pools()
    
    total_sim_rounds = int(np.ceil(scenario.TOTAL_SIM_TIME/scenario.ONEHOP_DELAY))
    input_time_counts = np.zeros((2*scenario.LANE_NUM_PER_DIRECT,),dtype=float)
    
    for time_flag in range(total_sim_rounds):
        
        input_time_counts = input_time_counts \
        + np.ones((2*scenario.LANE_NUM_PER_DIRECT,),dtype=float)*scenario.ONEHOP_DELAY
        
        scenario.update_position()
        scenario.communication()
        
        for lane_ID in range(scenario.LANE_NUM_PER_DIRECT):
            # right pool
            if input_time_counts[lane_ID] >= scenario.right_arrival_dt[lane_ID]:
                scenario.input_vehicle(1,lane_ID)
                input_time_counts[lane_ID] = 0.0
            if input_time_counts[lane_ID+2] >= scenario.left_arrival_dt[lane_ID]:
                scenario.input_vehicle(2,lane_ID)
                input_time_counts[lane_ID+2] = 0.0
#%% 
if __name__ == "__main__":
    test2()
                
