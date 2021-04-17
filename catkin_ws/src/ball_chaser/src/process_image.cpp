#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    
    ROS_INFO_STREAM("Robot Chasing The Ball");
    
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
        
    if (!client.call(srv)) {
	    ROS_ERROR("Failed");
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    float lin_x = 0.0;
    float ang_z = 0.0;
    float ball_pos = 0.0;
    int ball_pix = 0;

    for (int a = 0; a < img.height ; a++) {
        for (int b = 0; b < img.step; b+=3) {
            if (img.data[a * img.step + b] == white_pixel   &&  
		img.data[a * img.step + b+1] == white_pixel && 
                img.data[a * img.step + b+2] == white_pixel) {  
		ball_pos += b;
                ball_pix++;
            }
        }
    }

    
    if (ball_pix == 0 || 
        ball_pix > 0.15 * (img.height * img.step) ) {
        lin_x = 0.0;
        ang_z = 0.0;

    }
    else {
        lin_x = 0.1;
        ang_z = -0.5 * (ball_pos/ball_pix - img.step/2.0) / (img.step/2.0) ;
    }

    
    drive_robot(lin_x, ang_z);
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
