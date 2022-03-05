# Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its 
#    contributors may be used to endorse or promote products derived 
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from enum import Enum

from nanosaur_msgs.msg import Eyes
from nanosaur_msgs.srv import EyeMessage
from std_srvs.srv import Empty
from .display import Display, MessageType

class DisplayType(Enum):
    BOTH = 0
    LEFT = 1
    RIGHT = 2

DISPLAY_RATE = 5
class eyes:
    
    def __init__(self, node):
        self.node = node
        # Initialize right display parameters
        node.declare_parameter("display.right.enable", True)
        self.right_enable = bool(node.get_parameter("display.right.enable").value)
        node.declare_parameter("display.right.bus", 0)
        right_bus = int(node.get_parameter("display.right.bus").value)
        node.declare_parameter("display.right.address", 0x3C)
        right_address = int(node.get_parameter("display.right.address").value)
        # Initialize left display parameters
        node.declare_parameter("display.left.enable", True)
        self.left_enable = bool(node.get_parameter("display.left.enable").value)
        node.declare_parameter("display.left.bus", 1)
        left_bus = int(node.get_parameter("display.left.bus").value)
        node.declare_parameter("display.left.address", 0x3C)
        left_address = int(node.get_parameter("display.left.address").value)
        node.declare_parameter("display.sleep", 60 * 10)
        sleep_time = int(node.get_parameter("display.sleep").value)
        node.declare_parameter("display.splash", 3)
        splash_timeout = int(node.get_parameter("display.splash").value)
        
        # Initialize displays controllers
        if self.right_enable:
            node.get_logger().info(f"Display right bus={right_bus} adr={right_address} rate={DISPLAY_RATE}hz")
            self.display_right = Display(node, "right", i2c_bus=right_bus, i2c_address=right_address, rate=DISPLAY_RATE)
        else:
            node.get_logger().warn(f"Display right disabled")
        if self.left_enable:
            node.get_logger().info(f"Display left bus={left_bus} adr={left_address} Rate={DISPLAY_RATE}hz")
            self.display_left = Display(node, "left", i2c_bus=left_bus, i2c_address=left_address, rate=DISPLAY_RATE)
        else:
            node.get_logger().warn(f"Display left disabled")
            
        # Timeout timer
        self.timer = self.node.create_timer(sleep_time, self.timeout_callback)
        
        # Load spash screen
        if splash_timeout > 0:
            message = [f"nano", "saur"]
            middle_index = len(message)//2
            self.display_right.setMessage(message[middle_index:], MessageType.WIDE, timeout=splash_timeout)
            self.display_left.setMessage(message[:middle_index], MessageType.WIDE, timeout=splash_timeout)
        # Service
        self.message_srv = node.create_service(EyeMessage, 'nanosaur/message', self.message_service)
        self.message_srv = node.create_service(Empty, 'nanosaur/diagnostic', self.message_diagnostic)
        # Enable eyes topic
        if self.right_enable or self.left_enable:
            self.subscription = node.create_subscription(
                Eyes,
                'eyes',
                self.eyes_callback,
                10)
            self.subscription  # prevent unused variable warning
        else:
            node.get_logger().warn(f"eyes callback disabled")

    def ping(self):
        if self.right_enable:
            self.display_right.restartDisplay()
        if self.left_enable:
            self.display_left.restartDisplay()

    def message_diagnostic(self, req, resp):
        # Reset timer display sleep
        self.timer.reset()
        if self.right_enable:
            self.display_right.showDiagnostic(timeout=5)
        if self.left_enable:
            self.display_left.showDiagnostic(timeout=5)
        return resp

    def message_service(self, req, resp):
        # Reset timer display sleep
        self.timer.reset()
        # Print message
        display = DisplayType(req.display)
        type = MessageType(req.type)
        timeout = req.timeout
        messages = req.message
        
        self.node.get_logger().debug(f"Type={type} - Display: {display} - Timeout: {timeout}")
        self.node.get_logger().debug(f"Message: {messages}")

        if DisplayType.BOTH:
            middle_index = len(messages)//2
            if self.right_enable:
                self.display_right.setMessage(messages[middle_index:], type, timeout=timeout)
            if self.left_enable:
                self.display_left.setMessage(messages[:middle_index], type, timeout=timeout)
            resp.done = self.right_enable and self.left_enable
        elif DisplayType.RIGHT:
            if self.right_enable:
                self.display_right.setMessage(messages, type, timeout=timeout)
            resp.done = self.right_enable
        elif DisplayType.LEFT:
            if self.left_enable:
                self.display_left.setMessage(messages, type, timeout=timeout)
            resp.done = self.left_enable
        else:
            self.node.get_logger().error(f"Wrong Display type: {type}")
            resp.done = False
        
        return resp

    def timeout_callback(self):
        if self.right_enable:
            self.display_right.standby()
        if self.left_enable:
            self.display_left.standby()
        self.node.get_logger().info(f"Timeout!")
        

    def eyes_callback(self, msg):
        # Reset timer display sleep
        self.timer.reset()
        # Decode message
        x = msg.position.x
        y = msg.position.y
        # self.node.get_logger().info(f"eye position {x} - {y}")
        # Send new Eyes position
        x_l = x # if x > 0 else x * 2/3
        x_r = x
        y_l = y
        y_r = y
        if self.right_enable:
            self.display_right.setPoint(x_r, y_r)
        if self.left_enable:
            self.display_left.setPoint(x_l, y_l)
# EOF
