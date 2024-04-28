import rosbag
import csv

# bag_file_path = "../../short_range_rtk_2024-01-28-14-32-22.bag"

# bag = rosbag.Bag(bag_file_path)

# topics = ["/left_gps_driver/fix", "/right_gps_driver/fix"]

# csv_file_path = "stationary.csv"

# with open(csv_file_path, "w", newline="") as csv_file:
#     # Create a CSV writer
#     csv_writer = csv.writer(csv_file)

#     csv_writer.writerow(["GPS Type", "Timestamp", "Latitude", "Longitude", "Altitude"])

#     for i in topics:
#         for _, msg, timestamp in bag.read_messages(topics=i):
#             # Extract relevant data from the message
#             timestamp_secs = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
#             latitude = msg.latitude
#             longitude = msg.longitude
#             altitude = msg.altitude
#             csv_writer.writerow([i, timestamp_secs, latitude, longitude, altitude])

# bag.close()


# bag_file_path = "../../short_range_rtk_2024-01-28-14-32-22.bag"

# bag = rosbag.Bag(bag_file_path)

# topic = "/linearized_pose"

# csv_file_path = "pose.csv"

# with open(csv_file_path, "w", newline="") as csv_file:
#     csv_writer = csv.writer(csv_file)

#     csv_writer.writerow(
#         [
#             "Timestamp",
#             "Position_X",
#             "Position_Y",
#             "Position_Z",
#             "Orientation_X",
#             "Orientation_Y",
#             "Orientation_Z",
#             "Orientation_W",
#         ]
#     )

#     for _, msg, timestamp in bag.read_messages(topics=topic):
#         timestamp_secs = msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9
#         pos_x = msg.pose.pose.position.x
#         pos_y = msg.pose.pose.position.y
#         pos_z = msg.pose.pose.position.z
#         ori_x = msg.pose.pose.orientation.x
#         ori_y = msg.pose.pose.orientation.y
#         ori_z = msg.pose.pose.orientation.z
#         ori_w = msg.pose.pose.orientation.w

#         csv_writer.writerow([timestamp_secs, pos_x, pos_y, pos_z, ori_x, ori_y, ori_z, ori_w])

# bag.close()

bag_file_path = "../../short_range_rtk_2024-01-28-14-32-22.bag"

bag = rosbag.Bag(bag_file_path)

topics = ["/left_gps_driver/rtk_fix_status", "/right_gps_driver/rtk_fix_status"]

csv_file_left = "left_fix_status.csv"
csv_file_right = "right_fix_status.csv"

with open(csv_file_left, "w", newline="") as csv_file:
    csv_writer = csv.writer(csv_file)

    csv_writer.writerow(["Topic Name", "RTK_fix"])

    for _, msg, _ in bag.read_messages(topics="/left_gps_driver/rtk_fix_status"):
        rtk_fix_type = int(msg.RTK_FIX_TYPE)
        csv_writer.writerow(["/left_gps_driver/rtk_fix_status", rtk_fix_type])

with open(csv_file_right, "w", newline="") as csv_file:
    csv_writer = csv.writer(csv_file)

    csv_writer.writerow(["Topic Name", "RTK_fix"])

    for _, msg, _ in bag.read_messages(topics="/right_gps_driver/rtk_fix_status"):
        rtk_fix_type = int(msg.RTK_FIX_TYPE)
        csv_writer.writerow(["/right_gps_driver/rtk_fix_status", rtk_fix_type])

bag.close()
