import pyrealsense2 as rs

context = rs.context()
devices = context.query_devices()
for device in devices:
    print(f"Device: {device.get_info(rs.camera_info.name)}")
    print(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")