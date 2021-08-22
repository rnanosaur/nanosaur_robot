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

from nanosaur_msgs.msg import Eyes
from sensor_msgs.msg import Joy
from .display import Display

class eyes:
    
    def __init__(self, node):
        self.node = node
        # Initialize right Display controller
        node.declare_parameter("display.right.bus", 0)
        right_bus = int(node.get_parameter("display.right.bus").value)
        node.declare_parameter("display.right.address", 0x3C)
        right_address = int(node.get_parameter("display.right.address").value)
        node.declare_parameter("display.left.bus", 1)
        left_bus = int(node.get_parameter("display.left.bus").value)
        node.declare_parameter("display.left.address", 0x3C)
        left_address = int(node.get_parameter("display.left.address").value)
        
        timer_period = 0.25
        # Initialize Displays controller
        node.get_logger().info(f"Display left bus={left_bus} adr={left_address}")
        self.display_left = Display(node, i2c_bus=left_bus, i2c_address=left_address, timer_period=timer_period)
        node.get_logger().info(f"Display right bus={right_bus} adr={right_address}")
        self.display_right = Display(node, i2c_bus=right_bus, i2c_address=right_address, timer_period=timer_period)
        
        self.subscription = node.create_subscription(
            Eyes,
            'eyes',
            self.eyes_callback,
            10)
        self.subscription  # prevent unused variable warning
    
    def eyes_callback(self, msg):
        x = msg.x
        y = msg.y
        # self.node.get_logger().info(f"eye position {x} - {y}")
        # Send new Eyes position
        x_l = x # if x > 0 else x * 2/3
        x_r = x
        y_l = y
        y_r = y
        self.display_left.setPoint(x_l, y_l)
        self.display_right.setPoint(x_r, y_r)
# EOF
