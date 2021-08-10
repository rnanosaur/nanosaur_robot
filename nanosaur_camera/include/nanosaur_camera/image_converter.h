/**
 * Copyright (C) 2021, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

#include <jetson-utils/cudaUtility.h>
#include <jetson-utils/imageFormat.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

/**
 * GPU image conversion
 */
class imageConverter
{
public:
	/**
	 * Output image pixel type
	 */
	typedef float4 PixelType;

	/**
	 * Image format used for internal CUDA processing
	 */
	static const imageFormat InternalFormat = IMAGE_RGBA32F;

	/**
	 * Image format used for outputting ROS image messages
	 */
	static const imageFormat ROSOutputFormat = IMAGE_BGR8;

	/**
	 * Constructor
	 */
	imageConverter();

	/**
	 * Destructor
	 */
	~imageConverter();

	/**
	 * Free the memory
	 */
	void Free();

	/**
	 * Convert to 32-bit RGBA floating point
	 */
	bool Convert( const sensor_msgs::msg::Image::ConstSharedPtr& input );

	/**
	 * Convert to ROS sensor_msgs::Image message
	 */
	bool Convert( sensor_msgs::msg::Image& msg_out, imageFormat outputFormat );

	/**
	 * Convert to ROS sensor_msgs::Image message
	 */
	bool Convert( sensor_msgs::msg::Image& msg_out, imageFormat outputFormat, PixelType* imageGPU );

	/**
	 * Resize the memory (if necessary)
	 */
	bool Resize( uint32_t width, uint32_t height, imageFormat inputFormat );

	/**
	 * Retrieve the converted image width
	 */
	inline uint32_t GetWidth() const		{ return mWidth; }

	/**
	 * Retrieve the converted image height
	 */
	inline uint32_t GetHeight() const		{ return mHeight; }

	/**
	 * Retrieve the GPU pointer of the converted image
	 */
	inline PixelType* ImageGPU() const		{ return mOutputGPU; }

private:

	uint32_t mWidth;
	uint32_t mHeight;	
	size_t   mSizeInput;
	size_t   mSizeOutput;

	void* mInputCPU;
	void* mInputGPU;

	PixelType* mOutputCPU;
	PixelType* mOutputGPU;
};

#endif // IMAGE_CONVERTER_H