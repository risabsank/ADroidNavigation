#ifndef MAVLINK_COMM_H
#define MAVLINK_COMM_H

#include <iostream>
#include <unistd.h>  // For sleep function
#include <fcntl.h>   // For serial communication
#include <termios.h> // For configuring serial port
#include <cstring>
#include "mavlink_library/all/mavlink.h" // MAVLink header file

// MAVLink system configuration
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1
#define MAVLINK_TARGET_SYSTEM 1
#define MAVLINK_TARGET_COMPONENT 1

// Communication channel for MAVLink
#define MAVLINK_COMM_CHANNEL MAVLINK_COMM_0

// Define constants for navigating waypoints
#define NAV_CMD_WAYPOINT 16
#define FRAME_GLOBAL_RELATIVE_ALT 3
#define ALTITUDE 10.0f // Altitude in meters

// Serial device and baudrate
const int BAUDRATE = 0;
const char *DEVICE = ""; // Serial device connected to the flight controller

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller

Constructs a MAVLink heartbeat message, which is crucial for maintaining a communication link with the drone
Uses the mavlink_msg_heartbeat_pack function to create the message with relevant parameters
and then serializes it for transmission
*/
void send_heartbeat(int fd);

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller
@param count: total number of waypoints in the mission

Constructs a MAVLink mission count message and sends the serialized message through the specified serial port
*/
void send_mission_count(int fd, int count);

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller
@param lat: the latitude of the waypoint
@param lon: longitude of the waypoint
@param alt: altitude of the waypoint
@param seq: sequence number of the waypoint

Constructs a MAVLink waypoint message containing specific information about a waypoint in a drone's mission
Populates the waypoint structure with necessary details, encodes this information into a MAVLink message
*/
void send_waypoint(int fd, float lat, float lon, float alt, uint16_t seq);

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller
@return boolean if acknowledged or not

Helps confirm that the flight controller has received and understood the commands or waypoints sent by the program
*/
bool wait_for_ack(int fd);

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller

Serves as a trigger to begin executing the mission that has been defined by the waypoints sent previously
*/
void start_mission(int fd);

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller

Critical for changing the operational mode of the drone to AUTO mode
Ensures that the drone can transition from manual control or other modes to AUTO mode
*/
void set_mode_auto(int fd);

/*
@param fd: represents the file descriptor for the open serial port connection to the flight controller

Coordinating the drone's navigation through a series of predefined POIs
Ensures that the drone is properly configured to autonomously navigate the mission
Maintain a smooth communication flow between the ground station and the drone
*/
void navigate_through_pois(int fd);

#endif // MAVLINK_COMM_H