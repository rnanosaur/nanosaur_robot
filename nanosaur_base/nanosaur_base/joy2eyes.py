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
from rclpy.node import Node
import rclpy

class Joy2Eyes(Node):
    
    def __init__(self):
        super().__init__('Joy2Eyes')
        self.declare_parameter("axes.x", 3)
        self.axes_x = int(self.get_parameter("axes.x").value)
        self.declare_parameter("axes.y", 4)
        self.axes_y = int(self.get_parameter("axes.y").value)
        
        self.publisher_ = self.create_publisher(Eyes, 'eyes', 1)
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
        axes = msg.axes
        eyes_x = axes[self.axes_x] * 100.
        eyes_y = axes[self.axes_y] * 100.
        # Check if the message is the same, if true return and don't send the same message
        if (eyes_x == self.eyes_msg.x) or (eyes_y == self.eyes_msg.y):
            return
        # Read axis
        self.eyes_msg.x = eyes_x
        self.eyes_msg.y = eyes_y
        # Wrap to Eyes message
        self.publisher_.publish(self.eyes_msg)
        # Log message
        # self.get_logger().info(f"x {eyes_msg.x} y {eyes_msg.y}")


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
