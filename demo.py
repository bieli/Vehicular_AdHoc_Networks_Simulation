# -*- coding: utf-8 -*-
"""
Copyright (C) Thu Aug 25 00:11:08 2016  Jianshan Zhou
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
 
This module builds a simulation demo of VANETs based on epidemic routing.
"""
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt


from vehicle import Mobility
from scenario import Scenario

# simulation demo

#------------------------------------------------------------------------------
# set up the scenario
scenario = Scenario(Mobility(),
                 road_length = 2.0*10**3, # the length of the road in meter
                 two_direction = True, # bool-type flag indicates the directions
                 lane_width = 3.0, # lane width
                 lane_num_per_direct = 1, # the number of lanes per direction
                 vehicle_length = 5.0, # length of a vehicle in meter
                 onehop_delay = 10.0*10**(-3), # the delay in one-hop communication in second
                 total_sim_time = 60.0, # total simulation time in second
                 scenario_flag = "Freeway_Free",
                 commR = 50.0)
scenario.setup_vehicle_pools()


#------------------------------------------------------------------------------
# set up basic plot
total_sim_rounds = int(np.ceil(scenario.TOTAL_SIM_TIME/scenario.ONEHOP_DELAY))
input_time_counts = np.zeros((2*scenario.LANE_NUM_PER_DIRECT,),dtype=float)

dt = scenario.ONEHOP_DELAY

fig = plt.figure()
#fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
#ax = fig.add_subplot(111, aspect='equal', \
#                     autoscale_on=True,\
#                     xlim=(-1, scenario.ROAD_LENGTH+1), \
#                     ylim=(-1-scenario.LANE_NUM_PER_DIRECT*scenario.LANE_WIDTH,\
#                     1+scenario.LANE_NUM_PER_DIRECT*scenario.LANE_WIDTH))
deDist = 10.0
ax = fig.add_subplot(211, aspect = 20/1,\
                     xlim=(-deDist, scenario.ROAD_LENGTH+deDist), \
                     ylim=(-deDist-scenario.LANE_NUM_PER_DIRECT*scenario.LANE_WIDTH,\
                     deDist+scenario.LANE_NUM_PER_DIRECT*scenario.LANE_WIDTH))
# text object displaying time
time_text = ax.text(0.02, 0.8, '', fontsize=14, transform=ax.transAxes)

# particles holds the locations of the particles
ms = 5
particles1, = ax.plot([], [], 'bo', ms=ms)#for non-message carriers
particles2, = ax.plot([],[], 'ro', ms=ms)#for message carriers

#plot the road bounds and lanes
bounds1 = np.array([0.0,
                    scenario.ROAD_LENGTH,
                    -scenario.LANE_NUM_PER_DIRECT*scenario.LANE_WIDTH,
                    0.0])
bounds2 = np.array([0.0,
                    scenario.ROAD_LENGTH,
                    0.0,
                    scenario.LANE_NUM_PER_DIRECT*scenario.LANE_WIDTH])
rect1 = plt.Rectangle(bounds1[::2],
                     bounds1[1] - bounds1[0],
                     bounds1[3] - bounds1[2],
                     ec='none', lw=2, fc='none')
ax.add_patch(rect1)
rect2 = plt.Rectangle(bounds2[::2],
                     bounds2[1] - bounds2[0],
                     bounds2[3] - bounds2[2],
                     ec='none', lw=2, fc='none')
ax.add_patch(rect2)

# other subplots
ax2 = fig.add_subplot(223,adjustable='box',
                      xlim=(0.0,scenario.TOTAL_SIM_TIME),
                      ylim=(0.0,1.0))
ax3 = fig.add_subplot(224,
                      xlim=(0.0,scenario.TOTAL_SIM_TIME),
                      ylim=(0.0,scenario.ROAD_LENGTH))
                      
line2, = ax2.plot([], [], lw=3.0,color='r')
ax2.grid()
ax2.set_xlabel('simulation time (s)',fontsize=14)
ax2.set_ylabel('ratio of message carriers',fontsize=14)
xdata2, ydata2 = [], []

line3, = ax3.plot([], [], lw=3.0,color='b')
ax3.grid()
ax3.set_xlabel('simulation time (s)',fontsize=14)
ax3.set_ylabel('propagation distance (m)',fontsize=14)
xdata3, ydata3 = [], []
#------------------------------------------------------------------------------
# animation functions
def init():
    """initialize animation"""
    global scenario, rect1, rect2, xdata2, ydata2, xdata3, ydata3
    
    particles1.set_data([], [])
    particles2.set_data([],[])
    rect1.set_edgecolor('none')
    rect2.set_edgecolor('none')
    time_text.set_text('')
    xdata2.append(0.0)
    xdata3 = xdata2
    ydata2.append(scenario.infected_ratio)
    ydata3.append(scenario.propagation_distance)
    line2.set_data(xdata2,ydata2)
    line3.set_data(xdata3,ydata3)    
    
    return particles1, particles2, rect1, rect2, time_text, line2, line3

def animate(i):
    """perform animation step"""
    global scenario, rect1, rect2, ax, fig, input_time_counts, ms,\
    xdata2, ydata2, xdata3, ydata3
    
    # update the scenario state
    input_time_counts = input_time_counts \
            + np.ones((2*scenario.LANE_NUM_PER_DIRECT,),dtype=float)*scenario.ONEHOP_DELAY    
    
    scenario.update_position()
    scenario.communication()

    for lane_ID in range(scenario.LANE_NUM_PER_DIRECT):
        # right pool
        if input_time_counts[lane_ID] >= scenario.right_arrival_dt[lane_ID]:
            scenario.input_vehicle(1,lane_ID)
            input_time_counts[lane_ID] = 0.0
        if input_time_counts[lane_ID+scenario.LANE_NUM_PER_DIRECT] >= scenario.left_arrival_dt[lane_ID]:
            scenario.input_vehicle(2,lane_ID)
            input_time_counts[lane_ID+scenario.LANE_NUM_PER_DIRECT] = 0.0

    non_carrier_position = []
    carrier_position = []
    for lane_ID in range(scenario.LANE_NUM_PER_DIRECT):
        for vehicle in scenario.right_pool[lane_ID]:
            if vehicle.communication_terminal.received_flag:
                carrier_position.append([vehicle.position[0],vehicle.position[1]])
            else:
                non_carrier_position.append([vehicle.position[0],vehicle.position[1]])
        for vehicle in scenario.left_pool[lane_ID]:
            if vehicle.communication_terminal.received_flag:
                carrier_position.append([vehicle.position[0],vehicle.position[1]])
            else:
                non_carrier_position.append([vehicle.position[0],vehicle.position[1]])            
    non_carrier_position = np.asarray(non_carrier_position,dtype=float)
    carrier_position = np.asarray(carrier_position,dtype=float)
    
#    ms = int(fig.dpi * 2 * (scenario.VEHICLE_LENGTH*0.5) * fig.get_figwidth()
#             / np.diff(ax.get_xbound())[0])

    # update pieces of the animation
    rect1.set_edgecolor('k')
    rect2.set_edgecolor('k')
    
    particles1.set_data(non_carrier_position[:,0], non_carrier_position[:, 1])
    particles1.set_markersize(ms)
    particles2.set_data(carrier_position[:,0], carrier_position[:, 1])
    particles2.set_markersize(ms)    
    
    time_text.set_text('time = %.3fsec' % scenario.time_elapsed)
    
    xdata2.append(scenario.time_elapsed)
    ydata2.append(scenario.infected_ratio)
    xmin2, xmax2 = ax2.get_xlim()
    if scenario.time_elapsed >= xmax2:
        ax2.set_xlim(xmin2, 1.5*xmax2)
        ax2.figure.canvas.draw()    
    line2.set_data(xdata2,ydata2)
    
    ydata3.append(scenario.propagation_distance)
    xmin3, xmax3 = ax3.get_xlim()
    if scenario.time_elapsed >= xmax3:
        ax3.set_xlim(xmin3, 1.5*xmax3)
        ax3.figure.canvas.draw() 
    line3.set_data(xdata3,ydata3)      
    
    return particles1, particles2, rect1, rect2, time_text, line2, line3

#------------------------------------------------------------------------------
# do animation

# choose the interval based on dt and the time to animate one step
#from time import time
#t0 = time()
#animate(0)
#t1 = time()
#interval = 1000 * dt - (t1 - t0)
interval = 9 #ms
ani = animation.FuncAnimation(fig, animate, frames=total_sim_rounds,
                              interval=interval, blit=True, init_func=init)

plt.show()

ani = SubplotAnimation()
#ani.save('test_sub.mp4')
plt.show()
