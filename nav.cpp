// Descriptor Document: https://docs.google.com/document/d/1gUAp_euTT8lsyiKdfmtHqxED-qPsrNvXniLaUIFrYVI/edit?usp=sharing

#include <iostream>
#include <unistd.h>  // For sleep function
#include <fcntl.h>   // For serial communication
#include <termios.h> // For configuring serial port
#include <cstring>
#include "mavlink_library/all/mavlink.h" // MAVLink header file
#include "nav.h"

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
const int BAUDRATE = 115200;
const char *DEVICE = ""; // Serial device connected to the flight controller

// Function to send MAVLink heartbeat message
void send_heartbeat(int fd)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        MAV_TYPE_QUADROTOR, // Assuming a quadrotor drone, CHANGE IF NEEDED
        MAV_AUTOPILOT_GENERIC,
        MAV_MODE_MANUAL_ARMED, // Mode (can be adjusted)
        0,
        MAV_STATE_ACTIVE // Drone is active
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(fd, buf, len); // Write to serial port
}

// Function to send the total number of waypoints
void send_mission_count(int fd, int count)
{
    mavlink_message_t msg;
    mavlink_msg_mission_count_pack(
        MAVLINK_SYSTEM_ID,        // System ID
        MAVLINK_COMPONENT_ID,     // Component ID
        &msg,                     // Message pointer
        MAVLINK_TARGET_SYSTEM,    // Target system ID
        MAVLINK_TARGET_COMPONENT, // Target component ID
        count,                    // Number of waypoints
        MAV_MISSION_TYPE_MISSION, // Mission type
        0                         // Opaque ID, set to 0 or another value as needed
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(fd, buf, len);
}

// Function to send a single waypoint (mission item)
void send_waypoint(int fd, float lat, float lon, float alt, uint16_t seq)
{
    mavlink_message_t message;
    mavlink_mission_item_t waypoint;

    waypoint.target_system = MAVLINK_TARGET_SYSTEM;
    waypoint.target_component = MAVLINK_TARGET_COMPONENT;
    waypoint.seq = seq;
    waypoint.frame = FRAME_GLOBAL_RELATIVE_ALT;
    waypoint.command = NAV_CMD_WAYPOINT;
    waypoint.current = (seq == 0) ? 1 : 0;
    waypoint.autocontinue = 1;
    waypoint.param1 = 0; // Hold time (in seconds)
    waypoint.param2 = 0; // Acceptance radius (in meters)
    waypoint.param3 = 0; // Pass radius (in meters)
    waypoint.param4 = 0; // Desired yaw angle
    waypoint.x = lat;    // Latitude
    waypoint.y = lon;    // Longitude
    waypoint.z = alt;    // Altitude

    mavlink_msg_mission_item_encode(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &message, &waypoint);

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &message);
    write(fd, buf, len); // Send the waypoint over the serial port
}

// Function to wait for an acknowledgment from the flight controller
bool wait_for_ack(int fd)
{
    mavlink_message_t msg;
    mavlink_status_t status;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    int len = read(fd, buffer, sizeof(buffer)); // Read from serial port
    for (int i = 0; i < len; i++)
    {
        if (mavlink_parse_char(MAVLINK_COMM_CHANNEL, buffer[i], &msg, &status))
        {
            if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK)
            {
                return true; // Acknowledgment received
            }
        }
    }
    return false; // No acknowledgment
}

// Function to start the mission after all waypoints are sent
void start_mission(int fd)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        MAVLINK_SYSTEM_ID,        // System ID
        MAVLINK_COMPONENT_ID,     // Component ID
        &msg,                     // Message pointer
        MAVLINK_TARGET_SYSTEM,    // Target system ID
        MAVLINK_TARGET_COMPONENT, // Target component ID
        MAV_CMD_MISSION_START,    // Command ID
        0,                        // Confirmation number
        0, 0, 0, 0, 0, 0, 0       // Command parameters (set to 0 if not used)
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(fd, buf, len); // Send the command to start the mission
}

// Function to set the drone's mode to AUTO (to execute the mission)
void set_mode_auto(int fd)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(
        MAVLINK_SYSTEM_ID,
        MAVLINK_COMPONENT_ID,
        &msg,
        MAVLINK_TARGET_SYSTEM,
        MAVLINK_TARGET_COMPONENT,
        MAV_CMD_DO_SET_MODE,
        0, 4, 0, 0, 0, 0, 0, 0 // Mode AUTO (4)
    );

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    write(fd, buf, len);
}

// Function to simulate navigation by sending waypoints
void navigate_through_pois(int fd)
{
    // Define Points of Interest (latitude, longitude)
    float pois[][2] = {

    };
    int poi_count = sizeof(pois) / sizeof(pois[0]);

    // Send mission count to the drone
    send_mission_count(fd, poi_count);

    // Send each waypoint
    for (int i = 0; i < poi_count; i++)
    {
        float lat = pois[i][0];
        float lon = pois[i][1];

        // Send a waypoint for each POI
        send_waypoint(fd, lat, lon, ALTITUDE, i);

        if (!wait_for_ack(fd))
        {
            std::cerr << "Failed to receive acknowledgment for waypoint " << i << std::endl;
            return;
        }

        // Wait briefly before sending the next waypoint
        sleep(2);
    }

    // Start the mission
    start_mission(fd);
}

int main(int argc, char *argv[])
{
    // Open the serial connection
    int fd = open(DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        std::cerr << "Failed to open serial port " << DEVICE << std::endl;
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, BAUDRATE);
    cfsetospeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    tcsetattr(fd, TCSANOW, &options);

    send_heartbeat(fd);

    // Set drone mode to AUTO
    set_mode_auto(fd);

    // Simulate navigation through Points of Interest (POIs)
    navigate_through_pois(fd);

    close(fd);
    return 0;
}