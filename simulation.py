# # from pymavlink import mavutil
# # connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')  # Connect to the simulation

# # # Send a takeoff command
# # altitude = 10  # Altitude in meters
# # connection.mav.command_long_send(
# #     connection.target_system, connection.target_component,
# #     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

# # from pymavlink import mavutil

# # # Connect to the vehicle
# # connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")

# # # Set the target system and component IDs
# # target_system = 1  # ID of the vehicle you want to send the message to
# # target_component = 1  # ID of the specific component

# # # Set the latitude and longitude coordinates
# # latitude = 47.123456  # Replace with your desired latitude
# # longitude = -122.654321  # Replace with your desired longitude

# # # Set the altitude and altitude type
# # altitude = 50  # Replace with your desired altitude in meters
# # # altitude_type = mavutil.mavlink.MAV_ALTITUDE_ABSOLUTE  # Use absolute altitude

# # # Set the target coordinates
# # target_coordinates = (latitude, longitude, altitude)
# # print("Okkkkkkkkkkkkkkkkkkkkk")
# # # Create and send the SET_POSITION_TARGET_GLOBAL_INT message
# # msg = connection.mav.set_position_target_global_int_encode(
# #     0,
# #     0,
# #     target_system,
# #     target_component,
# #     mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
# #     0b0000111111111000,  # Use position and yaw
# #     *target_coordinates,
# #     0,
# #     0,
# #     0,
# #     0,
# #     0,
# #     0,
# #     0
# # )
# # connection.send(msg)


# # from pymavlink import mavutil

# # # Connect to the vehicle
# # connection = mavutil.mavlink_connection("udp:127.0.0.1:14550")

# # # Set the target system and component IDs
# # target_system = 1  # ID of the vehicle you want to send the message to
# # target_component = 1  # ID of the specific component

# # # Set the latitude and longitude coordinates
# # latitude = 47.123456  # Replace with your desired latitude
# # longitude = -122.654321  # Replace with your desired longitude

# # # Set the altitude and altitude type
# # altitude = 50  # Replace with your desired altitude in meters
# # altitude_type = mavutil.mavlink.MAV_ALTITUDE_ABSOLUTE  # Use absolute altitude

# # # Set the target coordinates
# # target_coordinates = (latitude, longitude, altitude)

# # # Create and send the SET_POSITION_TARGET_GLOBAL_INT message
# # msg = connection.message_factory.set_position_target_global_int_encode(
# #     0,
# #     0,
# #     target_system,
# #     target_component,
# #     mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
# #     0b0000111111111000,  # Use position and yaw
# #     *target_coordinates,
# #     0,
# #     0,
# #     0,
# #     0,
# #     0,
# #     0,
# #     0
# # )

# # connection.write(msg)

# from pymavlink import mavutil

# # Connect to the vehicle
# connection = mavutil.mavlink_connection(
#     "tcp:127.0.0.1:5760"
# )  # tcp:127.0.0.1:5760 #udp:127.0.0.1:14550
# while True:
#     msg = connection.recv_match()
#     if msg:
#         # Handle the received message
#         print(msg)

# # Set the target system and component IDs
# target_system = 1  # ID of the vehicle you want to send the message to
# target_component = 1  # ID of the specific component

# # Set the latitude and longitude coordinates
# latitude = 47.123456  # Replace with your desired latitude
# longitude = -122.654321  # Replace with your desired longitude

# # Set the altitude and altitude type
# altitude = 50  # Replace with your desired altitude in meters
# # altitude_type = mavutil.mavlink.MAV_ALTITUDE_RELATIVE  # Use relative altitude

# # Set the target coordinates
# target_coordinates = (latitude, longitude, altitude)

# # Create and send the SET_POSITION_TARGET_GLOBAL_INT message
# msg = connection.message_factory.set_position_target_global_int_encode(
#     0,
#     0,
#     target_system,
#     target_component,
#     mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
#     0b0000111111111000,  # Use position and yaw
#     *target_coordinates,
#     0,
#     0,
#     0,
#     0,
#     0,
#     0,
#     0
# )

# connection.write(msg)


# # from pymavlink import mavutil
# # altitude = 50
# # # lat_int = 47.123456

# # master = mavutil.mavlink_connection("tcp:127.0.0.1:14550", baud=115200)
# # long_int = -122.654321
# # lat_int = 47.123456
# # master.mav.set_position_target_global_int_send(
# #                                     master.target_system,
# #                                     master.target_component,
# #                                     coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
# #                                     type_mask=0b0000111111111000,
# #                                     lat_int = long_int
# #                                     lat_int = lat_int
# #                                     # long_int = -122.654321
# #                                     alt=50,  # (x, y WGS84 frame pos - not used), z [m]
# #                                     vx=0,
# #                                     vy=0,
# #                                     vz=0,  # velocities in NED frame [m/s] (not used)
# #                                     afx=0,
# #                                     afy=0,
# #                                     afz=0,
# #                                     yaw=0,
# #                                     yaw_rate=0
# #                                     )


# ________________________________________________________________________________________________________________________________________________-


# from pymavlink import mavutil

# # Connect to the ArduPilot SITL instance
# connection = mavutil.mavlink_connection("udpin:127.0.0.1:5760")
# print(connection)

# # Main loop to receive messages
# while True:
#     try:
#         # print("1")
#         # Receive a message
#         # message = connection.recv_match()
#         message = connection.recv_msg()
#         if message:
#             # Handle the received message
#             print(message)
#     except KeyboardInterrupt:
#         # Exit the loop on keyboard interrupt
#         break

# # Close the connection
# connection.close()

# import os

# os.environ["MAVLINK20"] = "1"
# os.environ["MAVLINK_DIALECT"] = "ardupilotmega"
# from pymavlink import mavutil


# def wait_heartbeat(m):
#     """wait for a heartbeat so we know the target system IDs"""
#     print("Waiting for APM heartbeat")
#     m.wait_heartbeat()
#     print(
#         "Heartbeat from APM (system %u component %u)"
#         % (m.target_system, m.target_system)
#     )


# mav_conn = mavutil.mavlink_connection("tcp:127.0.0.1:5762")

# wait_heartbeat(mav_conn)

# mav_conn.mav.request_data_stream_send(
#     mav_conn.target_system,
#     mav_conn.target_component,
#     mavutil.mavlink.MAV_DATA_STREAM_ALL,
#     1,
#     1,
# )

# while True:
#     msg = mav_conn.recv_match(blocking=True)
#     if not msg:
#         continue
#     if msg.get_type() == "Mode":
#         print("Message: %s" % msg)

# __________________________________________________working code___________________________________________________________________________
# import time
# from math import degrees
# from pymavlink import mavutil

# # Set the connection parameters
# connection_string = "udp:127.0.0.1:14550"  # Update with your connection string

# # Create a MAVLink connection
# master = mavutil.mavlink_connection(connection_string)

# # Wait for the heartbeat message to be received
# while True:
#     msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
#     msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
#     # master.mav.param_request_list_send(master.target_system, master.target_component)
#     msg2 = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
#     msg3 = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
#     msg4 = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
#     msg5 = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)
#     if msg is not None:
#         print("Heartbeat received!")
#         print("Mode", master.flightmode)
#         print("Latitude:", msg1.lat / 1e7)  # Divide by 1e7 to get decimal degrees
#         print("Longitude:", msg1.lon / 1e7)  # Divide by 1e7 to get decimal degrees
#         altitude = msg1.alt / 1000.0  # Convert to meters
#         print("Altitude:", altitude)
#         # print(msg3.battery_remaining)
#         print(msg5)

#         # You can access other information from the heartbeat message
#         # by accessing the appropriate attributes of the 'msg' object
#     else:
#         print("Waiting for heartbeat...")
#     time.sleep(1)  # Adjust the delay as needed

# ____________________________________________________________________________________________________________________________________________________--

# import time
# from math import radians
# from pymavlink import mavutil

# # Set the connection parameters
# connection_string = "udp:127.0.0.1:14550"  # Update with your connection string

# # Create a MAVLink connection
# master = mavutil.mavlink_connection(connection_string)
# boot_time = time.time()
# lat = -35.36236382
# long = 149.16113429
# depth = None
# # Wait for the heartbeat message to be received
# while True:
#     msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
#     msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
#     msg2 = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
#     msg3 = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
#     msg4 = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)

#     if msg is not None:
#         print("Heartbeat received!")
#         print("Mode:", master.flightmode)
#         print("Latitude:", msg1.lat / 1e7)  # Divide by 1e7 to get decimal degrees
#         print("Longitude:", msg1.lon / 1e7)  # Divide by 1e7 to get decimal degrees
#         altitude = msg1.alt / 1000.0  # Convert to meters
#         print("Altitude:", altitude)
#         altitude = 20

# Send the drone to the target latitude and longitude
# if master.flightmode == "GUIDED":
#     master.mav.set_position_target_local_ned_send(
#         int(1e3 * (time.time() - boot_time)),  # ms since boot
#         master.target_system,  # system ID
#         master.target_component,  # component ID
#         9,  # coordinate frame
#         3576,  # type mask
#         0,
#         -10,
#         0,  # x, y, z position [m]
#         0,
#         0,
#         0,  # vx, vy, vz velocity [m/s]
#         0,
#         0,
#         0,  # afx, afy, afz acceleration [m/s^2]
#         0,
#         0,
#     )  # yaw, yaw rate [rad/s]
# GPS = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)

# if GPS is not None:
#     master.mav.set_position_target_global_int_send(
#         int(1e3 * (time.time() - boot_time)),  # ms since boot
#         master.target_system,
#         master.target_component,
#         coordinate_frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#         type_mask=0b0000111111111000,
#         lat_int=GPS.lat,
#         lon_int=GPS.lon,
#         alt=altitude,  # (x, y WGS84 frame pos - not used), z [m]
#         vx=0,
#         vy=0,
#         vz=0,  # velocities in NED frame [m/s] (not used)
#         afx=0,
#         afy=0,
#         afz=0,
#         yaw=0,
#         yaw_rate=0
#         # accelerations in NED frame [N], yaw, yaw_rate
#         #  (all not supported yet, ignored in GCS Mavlink)
#     )
#         # print("MOVE RIGHT")
#         target_altitude = 150.0

#         # Set position target message
#         msg10 = connection_string.message_factory.set_position_target_local_ned_encode(
#             0,  # time_boot_ms (not used)
#             0,  # target_system (not used)
#             0,  # target_component (not used)
#             mavutil.mavlink.MAV_FRAME_LOCAL_NED,
#             0b0000111111111000,  # type_mask - set position and yaw
#             0,
#             0,
#             0,  # x, y, z positions (not used)
#             0,
#             0,
#             0,  # x, y, z velocity (not used)
#             0,
#             0,
#             0,  # x, y, z acceleration (not used)
#             0,
#             0,
#             0,  # x, y, z jerk (not used)
#             0,  # yaw (not used)
#             0,  # yaw_rate (not used)
#         )

#         # Set the target altitude
#         msg10.z = target_altitude

#     else:
#         print("Waiting for heartbeat...")
#     time.sleep(1)  # Adjust the delay as needed

# import time
# from pymavlink import mavutil

# # Set the connection parameters
# connection_string = "udp:127.0.0.1:14550"  # Update with your connection string

# # Create a MAVLink connection
# master = mavutil.mavlink_connection(connection_string)

# # Wait for the heartbeat message to be received
# while True:
#     msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

#     if msg is not None:
#         print("Heartbeat received!")
#         print("Mode:", master.flightmode)

#         # Get the target latitude and longitude from the user
#         target_lat = float(
#             input("Enter target latitude: ")
#         )  # Example: 42.123456 (decimal degrees)
#         target_lon = float(
#             input("Enter target longitude: ")
#         )  # Example: -71.654321 (decimal degrees)
#         target_alt = 100.0  # Replace with your target altitude in meters

#         # Convert target latitude and longitude to degrees multiplied by 1e7
#         target_lat_deg = int(target_lat * 1e7)
#         target_lon_deg = int(target_lon * 1e7)

#         # Create the MISSION_ITEM_INT message
#         msg_mission_item = master.mav.mission_item_int_send(
#             master.target_system,
#             master.target_component,
#             0,
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#             mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
#             0,
#             1,
#             0,
#             0,
#             0,
#             0,get_static_pad
#             target_alt,
#         )

#         # Wait for the command to be acknowledged
#         while True:
#             ack_msg = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)
#             if (
#                 ack_msg is not None
#                 and ack_msg.target_system == master.target_system
#                 and ack_msg.target_component == master.target_component
#             ):
#                 if ack_msg.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
#                     print("Target waypoint set successfully!")
#                 else:
#                     print("Failed to set target waypoint!")
#                 break

#     else:
#         print("Waiting for heartbeat...")
#     time.sleep(1)  # Adjust the delay as needed

# _______________________________________working code________________________________________________________________________________
# import time
# from math import degrees
# from pymavlink import mavutil

# target_alt = 50
# # Set the connection parameters
# connection_string = "udp:127.0.0.1:14550"  # Update with your connection string

# # Create a MAVLink connection
# master = mavutil.mavlink_connection(connection_string)


# # Function to get the current target latitude and longitude (example implementation)
# def get_target_location():
#     # Example implementation: Get the target location from a GPS receiver or any other source
#     target_lat = -35.36  # Update with the actual target latitude
#     target_lon = 149.16901338  # Update with the actual target longitude
#     return target_lat, target_lon


# # Wait for the heartbeat message to be received
# while True:
#     msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
#     msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
#     msg2 = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
#     msg3 = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
#     msg4 = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
#     msg5 = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)

#     if msg is not None:
#         print("Heartbeat received!")
#         print("Mode:", master.flightmode)
#         print("Latitude:", msg1.lat / 1e7)  # Divide by 1e7 to get decimal degrees
#         print("Longitude:", msg1.lon / 1e7)  # Divide by 1e7 to get decimal degrees
#         altitude = msg1.alt / 1000.0  # Convert to meters
#         print("Altitude:", altitude)
#         print(msg5)
#         yaw = msg2.yaw
#         print("yawangleeeeeeeeeeeeeeee", yaw)

#         # You can access other information from the heartbeat message
#         # by accessing the appropriate attributes of the 'msg' object

#         # Request parameter list
#         master.mav.param_request_list_send(
#             master.target_system, master.target_component
#         )

#         # Get the current target latitude and longitude
#         target_lat, target_lon = get_target_location()

#         # Set the target system and component IDs
#         target_system = 1
#         target_component = 1

#         # Create a MAVLink message for follow me mode
#         msg = master.mav.set_position_target_global_int_send(
#             0,  # Time since boot (ms)
#             target_system,  # Target system ID
#             target_component,  # Target component ID
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Coordinate frame
#             0b0000111111111000,  # Type mask
#             int(target_lat * 1e7),  # Target latitude (scaled to integer)
#             int(target_lon * 1e7),  # Target longitude (scaled to integer)
#             target_alt,  # Target altitude
#             0,
#             0,
#             0,  # Velocity in NED frame (not used)
#             0,
#             0,
#             0,  # Acceleration (not used)
#             0,  # Yaw angle (not used)
#             0,  # Yaw rate (not used)
#         )
#         print(target_lat)

#     else:
#         print("Waiting for heartbeat...")

#     time.sleep(1)  # Adjust the delay as needed
# __________________________________________________working code___________________________________________________
# import time
# import random
# from math import degrees
# from pymavlink import mavutil

# # Set the connection parameters
# connection_string = "udp:127.0.0.1:14550"  # Update with your connection string

# # Create a MAVLink connection
# master = mavutil.mavlink_connection(connection_string)
# target_alt = 50


# # Function to get the current target latitude and longitude
# def get_target_location():
#     # Simulate changing values with random generator
#     target_lat = random.uniform(
#         47.0, 48.0
#     )  # Update with your actual source of target latitude
#     target_lon = random.uniform(
#         -123.0, -122.0
#     )  # Update with your actual source of target longitude
#     return target_lat, target_lon


# # Wait for the heartbeat message to be received
# while True:
#     msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
#     msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
#     msg2 = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
#     msg3 = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
#     msg4 = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
#     msg5 = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)

#     if msg is not None:
#         print("Heartbeat received!")
#         print("Mode:", master.flightmode)
#         print("Latitude:", msg1.lat / 1e7)  # Divide by 1e7 to get decimal degrees
#         print("Longitude:", msg1.lon / 1e7)  # Divide by 1e7 to get decimal degrees
#         altitude = msg1.alt / 1000.0  # Convert to meters
#         print("Altitude:", altitude)
#         print(msg5)

#         # You can access other information from the heartbeat message
#         # by accessing the appropriate attributes of the 'msg' object

#         # Request parameter list
#         master.mav.param_request_list_send(
#             master.target_system, master.target_component
#         )

#         # Get the current target latitude and longitude
#         target_lat, target_lon = get_target_location()

#         # Set the target system and component IDs
#         target_system = 1
#         target_component = 1

#         # Create a MAVLink message for follow me mode
#         msg = master.mav.set_position_target_global_int_send(
#             0,  # Time since boot (ms)
#             target_system,  # Target system ID
#             target_component,  # Target component ID
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Coordinate frame
#             0b0000111111111000,  # Type mask
#             int(target_lat * 1e7),  # Target latitude (scaled to integer)
#             int(target_lon * 1e7),  # Target longitude (scaled to integer)
#             target_alt,  # Target altitude
#             0,
#             0,
#             0,  # Velocity in NED frame (not used)
#             0,
#             0,
#             0,  # Acceleration (not used)
#             0,  # Yaw angle (not used)
#             0,  # Yaw rate (not used)
#         )

#     else:
#         print("Waiting for heartbeat...")

#     time.sleep(1)  # Adjust the delay as needed


# import time
# from math import degrees
# from pymavlink import mavutil

# # Set the connection parameters
# connection_string = "udp:127.0.0.1:14550"  # Update with your connection string

# # Create a MAVLink connection
# master = mavutil.mavlink_connection(connection_string)
# target_alt = 50

# # Define the list of target latitude and longitude values
# target_locations = [
#     (-35.36265576, 149.164039523),
#     (-35.36176870, 149.16293802),
#     (-35.36087040, 149.16182274),
#     (-35.35997208, 149.16073500),
#     (-35.35912349, 149.15973968),
# ]

# # Set the target system and component IDs
# target_system = 1
# target_component = 1

# # Index to keep track of the current target location
# target_index = 0

# # Wait for the heartbeat message to be received
# while True:
#     msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
#     msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
#     msg2 = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
#     msg3 = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
#     msg4 = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
#     msg5 = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)

#     if msg is not None:
#         print("Heartbeat received!")
#         print("Mode:", master.flightmode)
#         print("Latitude:", msg1.lat / 1e7)  # Divide by 1e7 to get decimal degrees
#         print("Longitude:", msg1.lon / 1e7)  # Divide by 1e7 to get decimal degrees
#         altitude = msg1.alt / 1000.0  # Convert to meters
#         print("Altitude:", altitude)
#         print(msg5)

#         # You can access other information from the heartbeat message
#         # by accessing the appropriate attributes of the 'msg' object

#         # Request parameter list
#         master.mav.param_request_list_send(
#             master.target_system, master.target_component
#         )

#         # Get the current target latitude and longitude
#         target_lat, target_lon = target_locations[target_index]

#         # Create a MAVLink message for follow me mode
#         msg = master.mav.set_position_target_global_int_send(
#             0,  # Time since boot (ms)
#             target_system,  # Target system ID
#             target_component,  # Target component ID
#             mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Coordinate frame
#             0b0000111111111000,  # Type mask
#             int(target_lat * 1e7),  # Target latitude (scaled to integer)
#             int(target_lon * 1e7),  # Target longitude (scaled to integer)
#             target_alt,  # Target altitude
#             0,
#             0,
#             0,  # Velocity in NED frame (not used)
#             0,
#             0,
#             0,  # Acceleration (not used)
#             0,  # Yaw angle (not used)
#             0,  # Yaw rate (not used)
#         )

#         # Increment the target index
#         target_index = (target_index + 1) % len(target_locations)
#     else:
#         print("Waiting for heartbeat...")

#     time.sleep(1)  # Adjust the delay as needed


import time
from math import degrees
from pymavlink import mavutil

# Set the connection parameters
connection_string = "udp:127.0.0.1:14550"  # Update with your connection string
boot_time = time.time()
# Create a MAVLink connection
master = mavutil.mavlink_connection(connection_string)
target_alt = 0
print("sathiya")

# Define the list of target latitude and longitude values
target_locations = [
    (-35.36265576, 149.164039523),
    (-35.36176870, 149.16293802),
    (-35.36089115, 149.15968646),
    (-35.35997208, 149.16073500),
    (-35.35912349, 149.15973968),
]
lat = -36.35912349
long = 140.15973968

# Set the target system and component IDs
target_system = 1
target_component = 1

# Wait for the heartbeat message to be received
while True:
    msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
    msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)
    msg2 = master.recv_match(type="ATTITUDE", blocking=True, timeout=1.0)
    msg3 = master.recv_match(type="SYS_STATUS", blocking=True, timeout=1.0)
    msg4 = master.recv_match(type="GPS_RAW_INT", blocking=True, timeout=1.0)
    msg5 = master.recv_match(type="MISSION_ACK", blocking=True, timeout=1.0)

    if msg is not None:
        print("Heartbeat received!")
        print("Mode:", master.flightmode)
        print("Latitude:", msg1.lat / 1e7)  # Divide by 1e7 to get decimal degrees
        print("Longitude:", msg1.lon / 1e7)  # Divide by 1e7 to get decimal degrees
        altitude = msg1.alt / 1000.0  # Convert to meters
        print("Altitude:", altitude)
        print(msg5)

        # You can access other information from the heartbeat message
        # by accessing the appropriate attributes of the 'msg' object

        # Request parameter list
        master.mav.param_request_list_send(
            master.target_system, master.target_component
        )

        # Check if all target locations have been used
        if target_locations:
            # Get the current target latitude and longitude
            target_lat, target_lon = target_locations.pop(0)

            # Create a MAVLink message for follow me mode
            # msg = master.mav.set_position_target_global_int_send(
            #     0,  # Time since boot (ms)
            #     target_system,  # Target system ID
            #     target_component,  # Target component ID
            #     mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Coordinate frame
            #     0b0000111111111000,  # Type mask
            #     int(target_lat * 1e7),  # Target latitude (scaled to integer)
            #     int(target_lon * 1e7),  # Target longitude (scaled to integer)
            #     target_alt,  # Target altitude
            #     0,
            #     0,
            #     0,  # Velocity in NED frame (not used)
            #     0,
            #     0,
            #     0,  # Acceleration (not used)
            #     0,  # Yaw angle (not used)
            #     0,  # Yaw rate (not used)
            # )
            lat = int(lat)
            long = int(long)
            print(lat, "latttttttttttttttttttttt")
            print(long, "longgggggggggggggggggggggg")
            # pymavlink command to steer the drone to the calculated lat, long
            msg = master.mav.set_position_target_global_int_send(
                int(1e3 * (time.time() - boot_time)),  # ms since boot
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # MAV_FRAME_GLOBAL_INT,coordinate_frame
                type_mask=0b0000111111111000,
                lat_int=lat,
                lon_int=long,
                alt=0,  # (x, y WGS84 frame pos - not used), z [m]
                vx=0,
                vy=0,
                vz=0,  # velocities in NED frame [m/s] (not used)
                afx=0,
                afy=0,
                afz=0,
                yaw=0,
                yaw_rate=0,
            )
            # accelerations in NED frame [N], yaw, yaw_rate
            #
        else:
            print("All target locations have been used.")
            break

    else:
        print("Waiting for heartbeat...")
    time.sleep(1)  # Adjust the delay as needed
