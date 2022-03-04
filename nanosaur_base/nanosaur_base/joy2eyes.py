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
from nanosaur_msgs.srv import EyeMessage
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from rclpy.node import Node
import rclpy

class Joy2Eyes(Node):
    
    def __init__(self):
        super().__init__('Joy2Eyes')
        self.declare_parameter("axes.x", 3)
        self.axes_x = int(self.get_parameter("axes.x").value)
        self.declare_parameter("axes.y", 4)
        self.axes_y = int(self.get_parameter("axes.y").value)
        self.declare_parameter("button", 8)
        self.b_message = int(self.get_parameter("button").value)
        self.declare_parameter("diagnostic", 7)
        self.b_diagnostic = int(self.get_parameter("diagnostic").value)
        
        self.publisher_ = self.create_publisher(Eyes, 'eyes', 1)
        
        self.cli_message = self.create_client(EyeMessage, 'nanosaur/message')
        self.cli_diagnostic = self.create_client(Empty, 'nanosaur/diagnostic')
        # Eyes message
        self.eyes_msg = Eyes()
        # Register subcriber
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1)
        self.subscription  # prevent unused variable warning
        
    def joy_callback(self, msg):
        # Decode joystick message
        # Decode buttons message
        buttons = msg.buttons
        
        # Check if pressed the centre button and send a test message
        if buttons[self.b_message]:
            req = EyeMessage.Request()
            req.display = req.BOTH
            req.type = req.WIDE
            req.timeout  = 3
            req.message = ["Hello", "World"]
            self.get_logger().info(f"request service message")
            future = self.cli_message.call_async(req)
            return
        elif buttons[self.b_diagnostic]:
            req = Empty.Request()
            self.get_logger().info(f"request service diagnostic")
            future = self.cli_diagnostic.call_async(req)
            return
        
        # Decode axes message
        axes = msg.axes
        eyes_x = axes[self.axes_x] * 100.
        eyes_y = axes[self.axes_y] * 100.
        # Check if the message is the same, if true return and don't send the same message
        if (eyes_x == self.eyes_msg.position.x) and (eyes_y == self.eyes_msg.position.y):
            return
        # Read axis
        self.eyes_msg.position.x = eyes_x
        self.eyes_msg.position.y = eyes_y
        # Wrap to Eyes message
        self.publisher_.publish(self.eyes_msg)
        # Log message
        # self.get_logger().info(f"x {self.eyes_msg.x} y {self.eyes_msg.y}")


def main(args=None):
    rclpy.init(args=args)

    wrapper = Joy2Eyes()
    try:
        rclpy.spin(wrapper)
    except KeyboardInterrupt:
        pass
    # Destroy the node explicitly
    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# EOF
