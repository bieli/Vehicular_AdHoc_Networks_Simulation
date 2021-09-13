# Vehicular AdHoc Networks Simulation
Copyright (C) Thu Aug 25 00:11:08 2016  Jianshan Zhou - upgraded after ~5yrs by `bieli`

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

In this ReadMe file, the basic components that make up this vehicular
 ad-hoc networks (VANETs) simulation are outlined.
 
#---------------------------
**communication_terminal.py**

  This file defines a basic class representing the vehicular communication terminal.

**vehicle.py**

  This file defines a vehicle class that consists of the IDM model and needs the support of the communication terminal class.

**scenario.py**

  This file provides the experimental settings on the vehicle speed distribution and the inter-vehicle distance distribution that can reflect different traffic situations. The scenario class defines some basic simulation components needed to initialize and update the overall vehicular network state. Specifically, the position update function and the interaction among neighboring vehicles are detailed. It is worth pointing out here that the current scenario can support the simulation of bi-directional multi-lane vehicle platoons.

**demo.py**

  This file provides a basic simulation animation demo for the application of this VANET simulation project.     

#---------------------------
The objective of this simulation project developed in Python is to explore implementation of a simple epidemic-based routing mechanism designed to support reliable message dissemination in vehicular ad-hoc networks. In this project, the simulation can be carried out under different mobility by adopting different distributions of vehicle speed and inter-vehicle space, which provides several typical traffic scenarios for studying the epidemic routing mechanism. In addition, I remark that the basic mobility model used here is based on the well-known intelligent dreiver model (IDM), the detailed mathematical representation of the IDM can be referred to in the WIKIPEDIA: https://en.wikipedia.org/wiki/Intelligent_driver_model, and the default mobility parameters involved are set according to existing liturature.

You are perfectly welcome to use and modify this simulation project, and contribute to improving it. But you should keep the original copyright information above in your own re-distribution. Thanks!
