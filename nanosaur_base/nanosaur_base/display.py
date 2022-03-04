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

# Display
# https://github.com/adafruit/Adafruit_Python_SSD1306
import Adafruit_SSD1306

import atexit
from enum import Enum
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

class MessageType(Enum):
    FULL = 0
    WIDE = 1

class DisplayStatus(Enum):
    SLEEP = 0
    RUNNING = 1
    MESSAGE = 2

# Install open-sans
# apt install fonts-open-sans
# List all fonts with: fc-list
FONT_NAME = 'OpenSans-Regular.ttf'
FONT_NAME_WIDE = 'OpenSans-Bold.ttf'

def circle(x_c, y_c, radius):
    x_min = x_c - radius
    y_min = y_c - radius
    x_max = x_c + radius
    y_max = y_c + radius
    return (x_min, y_min, x_max, y_max)

class Display:

    def __init__(self, node, name, rate=1, i2c_bus=1, i2c_address=0x3C):
        """
        Reference:
        - https://github.com/adafruit/Adafruit_Python_SSD1306/blob/master/examples/shapes.py
        """
        self.node = node
        # 128x32 display with hardware I2C:
        # setting gpio to 1 is hack to avoid platform detection
        self.disp = Adafruit_SSD1306.SSD1306_128_64(rst=None, i2c_bus=i2c_bus, gpio=1, i2c_address=i2c_address)
        # Init display
        self.disp.begin()
        # Clear display.
        self.disp.clear()
        self.disp.display()
        # Create blank image for drawing.
        # Make sure to create image with mode '1' for 1-bit color.
        self.width = self.disp.width
        self.height = self.disp.height
        self.image = Image.new('1', (self.width, self.height))
        # Get drawing object to draw on image.
        self.draw = ImageDraw.Draw(self.image)
        # Load default font
        try:
            self.font_default = ImageFont.truetype(FONT_NAME, 11)
            self.font_wide = ImageFont.truetype(FONT_NAME_WIDE, 54)
        except OSError:
            node.get_logger().error(f"Fail load {FONT_NAME}. Loaded default font")
            self.font_default = ImageFont.load_default()
            self.font_wide = ImageFont.load_default()
        # Draw some shapes.
        # First define some constants to allow easy resizing of shapes.
        padding = 2
        self.top = padding
        self.bottom = self.height-padding
        # status
        self.status_display = DisplayStatus.RUNNING
        # Show message
        self.counter = 0
        self.timeout_message = 0
        # Center eyes
        self.shape_width_big = 30
        self.center_x = 0
        self.center_y = 0
        # Message
        self.message = []
        self.type = MessageType(0)
        self.diagnostic = [f"Display: {name}", f"BUS: {i2c_bus}", f"ID: {i2c_address}", f"rate: {rate}hz", f"res={self.width}px x {self.height}px"]
        # Init display timer
        self.timer_period = 1.0 / rate
        self.timer = self.node.create_timer(self.timer_period, self.display_callback)
        # Initialize display
        self.setPoint()
        # Configure all motors to stop at program exit
        atexit.register(self._close)

    def setPoint(self, x=0, y=0):
        self.status_display = DisplayStatus.RUNNING
        '''
        Numbers between -100, 100
        '''
        # https://stackoverflow.com/questions/5996881/how-to-limit-a-number-to-be-within-a-specified-range-python
        clamp = lambda n, minn, maxn: max(min(maxn, n), minn)
        # limit between -100 and 100
        x = clamp(x, -100, 100)
        y = clamp(y + 50, -100, 100)
        
        self.center_x = (x / 100.) * (self.width) / 2.
        self.center_y = (y / 100.) * (self.height) / 2.

    def showDiagnostic(self, timeout=3):
        self.setMessage(self.diagnostic, MessageType.FULL, timeout=timeout)

    def setMessage(self, message, type, timeout=3.0):
        self.message = message
        self.timeout_message = timeout
        self.type = MessageType(type)
        self.counter = 0
        self.status_display = DisplayStatus.MESSAGE
   
    def _print_eye(self):
        center_x = self.center_x + self.width / 2.
        center_y = self.center_y + self.height / 2.
        
        center_big_x = 0 if abs(self.center_x) < self.shape_width_big else self.center_x / 2.
        center_big_y = 0 if abs(self.center_y) < self.shape_width_big else self.center_y / 2.

        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        
        self.draw.ellipse(circle(center_big_x + self.width / 2., center_big_y + self.height / 2., self.shape_width_big), outline=255, fill=0)
        self.draw.ellipse(circle(center_x, center_y, 10), outline=255, fill=1)

        # Line separation
        #self.draw.line((self.width * 1/4, 0, self.width* 1/4, self.height), fill=255)
        #self.draw.line((self.width/2, 0, self.width/2, self.height), fill=255)
        #self.draw.line((self.width * 3/4, 0, self.width* 3/4, self.height), fill=255)

    def _print_message(self):
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        # Draw with type message
        if self.type == MessageType.FULL:
            # Draw all lines
            step = -2
            for idx, message in enumerate(self.message):
                (font_width, font_height) = self.font_default.getsize(message)
                self.draw.text((0, step), message, font=self.font_default, fill=255)
                step += font_height
        else:
            self.draw.text((0, -10), self.message[0], font=self.font_wide, fill=255)

    def restartDisplay(self):
        if self.counter == 0:
            self.status_display = DisplayStatus.RUNNING

    def standby(self):
        # Clean display
        self.status_display = DisplayStatus.SLEEP 

    def display_callback(self):
        if self.status_display == DisplayStatus.MESSAGE or self.counter > 0:
            self._print_message()
            self.counter += self.timer_period
            # Reset counter
            if self.counter > self.timeout_message:
                self.status_display = DisplayStatus.RUNNING
                self.counter = 0
        elif self.status_display == DisplayStatus.RUNNING:
            self._print_eye()
        else:
            # Draw a black filled box to clear the image.
            self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)  
        # Show image on screen
        self.disp.image(self.image)
        self.disp.display()
    
    def _close(self):
        # Draw a black filled box to clear the image.
        self.draw.rectangle((0, 0, self.width, self.height), outline=0, fill=0)
        self.disp.image(self.image)
        self.disp.display()
# EOF
