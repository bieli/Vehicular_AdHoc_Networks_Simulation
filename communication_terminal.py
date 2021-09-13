# -*- coding: utf-8 -*-
"""
Copyright (C) Mon Aug 22 16:20:48 2016  Jianshan Zhou
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
 
This module defines the class of the vehicular communication terminal.
"""


class Communication_ternimal(object):
    def __init__(
        self, ID, received_flag=False  # terminal ID
    ):  # flag indicating whether receiving a message
        self.ID = ID
        self.received_flag = received_flag
        self.buffer_max_length = 100
        self.message_buffer = []

    def receive(self, message):
        if len(self.message_buffer) < self.buffer_max_length:
            self.message_buffer.append(message)

        if self.message_buffer:
            self.received_flag = True
        else:
            self.received_flag = False

    def update_buffer(self):
        # more complicated buffering protocol should be done in future
        pass

    def send(self, neighboring_node):
        for message in self.message_buffer:
            neighboring_node.communication_terminal.receive(message)
