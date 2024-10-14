```
// MAVLink system configuration
#define MAVLINK_SYSTEM_ID 1
#define MAVLINK_COMPONENT_ID 1
#define MAVLINK_TARGET_SYSTEM 1
#define MAVLINK_TARGET_COMPONENT 1
```
Define the system and component ids as well as the target system and component where messages throughout the drone’s navigation will be sent to. We essentially have one system/component right now.

```
#define MAVLINK_COMM_CHANNEL MAVLINK_COMM_0
```
Corresponds to the channel of communication.

```
// Define constants for navigating waypoints
#define NAV_CMD_WAYPOINT 16
#define FRAME_GLOBAL_RELATIVE_ALT 3
#define ALTITUDE 10.0f // Altitude in meters
https://mavlink.io/en/messages/common.html
```
Value of 16 corresponds to the MavLink command ID for a waypoint for the drone’s mission. The value of 3 for the FRAME_GLOBAL_RELATIVE_ALT specifies that the coordinates are in a global frame with relative altitude. The altitude is going to be the altitude of the drone. I have inputted 10 as a fill-in value and it can be changed.

```
// Serial device and baudrate
const int BAUDRATE = 115200;
const char *DEVICE = ""; // Serial device connected to the flight controller
```
Here, we initialize a value for the baud rate for serial communication and declare a constant string for the serial device name.

```
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
```
Calls mavlink_msg_heartbeat_pack to fill the msg variable:
MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, msg, MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, MAV_MODE_MANUAL_ARMED, 0, MAV_STATE_ACTIVE
Creates buffer to hold serialized message data and sends the serialized message over with a secure connection

```
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
```
Calls mavlink_msg_mission_count_pack to send information about number of waypoints that need to processed for the mission
Creates buffer to hold serialized message data, packs buffer, then sends serialized message to specific serial port

```
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
```
Sets the fields of the waypoint structure:
Target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z
Encodes waypoint information into MavLink message that gets sent over the serial connection

```
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
```
Read data from the serial port into the buffer
Each byte is processed with the mavlink_parse_char() function, which checks if the byte is part of a complete Mavlink message 
Inside the loop we check if the message has been received or not and return whether this is true or false accordingly

```
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
```
Construct a mavlink_msg_command_long_pack which utilizes information regarding the drone → send the serialized message via serial port
The message that is sent helps move the drone from position of preparation to beginning its flight

```
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
```
Shifts the drone’s mode from manual to auto and sends the message over the serial port

```
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
```
Calculate number of POIs and send the number of waypoints through send_mission_count()
Iterates through each POI, retrieves its latitude and longitude, and calls the send_waypoint() function to send each waypoint to the drone → Checks for an acknowledgement from the drone using wait_for_ack()
Include delays to ensure for smooth communication
Start mission once all waypoints are processed

```
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
```
First, attempts to open a serial port defined by the DEVICE constant
Store current options for the terminal interface
Establishing a reliable serial connection between your application and the drone's flight controller
Calls the send_heartbeat() function
Set the drone’s mode to AUTO and begin sending waypoints through navigate_through_pois
