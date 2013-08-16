/*
 * kinect_teleop_keyboard.cpp
 *
 *  Created on: Jun 11, 2012
 *      Author: vittorio
 *      Description: This programs recieves input from keyboard in terminal, and publishes to tilt angle to move kinect.
 *
 */
/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread/thread.hpp>


#define KEYCODE_U 0x41
#define KEYCODE_D 0x42


class kinect_teleop_keyboard
{
public:
	kinect_teleop_keyboard();
  void keyLoop();

private:


  ros::NodeHandle nh_,ph_;
  ros::Publisher kinectAnglePublisher;
  std_msgs::Float64 kinectAngle;

};

kinect_teleop_keyboard::kinect_teleop_keyboard()
{
	kinectAngle.data = 0.0; // set initial angle to 0
	kinectAnglePublisher = nh_.advertise<std_msgs::Float64>("tilt_angle", 30);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_teleop_keyboard");
  kinect_teleop_keyboard kinect_teleop_keyboard_OBJECT;
  ros::NodeHandle n;

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&kinect_teleop_keyboard::keyLoop, &kinect_teleop_keyboard_OBJECT));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join() ;

  return(0);
}




void kinect_teleop_keyboard::keyLoop()
{
  char c;


  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the kinect.");


  while (ros::ok())
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c)
    {
      case KEYCODE_U:
        ROS_DEBUG("UP");
        kinectAngle.data = kinectAngle.data + 0.5;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        kinectAngle.data = kinectAngle.data - 0.5;
        break;
    }

    kinectAnglePublisher.publish(kinectAngle);

  }
  return;

}

