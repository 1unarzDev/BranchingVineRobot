#!/usr/bin/env python3

import asyncio
import websockets
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from concurrent.futures import ThreadPoolExecutor

class WebSocketNode(Node):
    def __init__(self):
        super().__init__('websocket_node')
        self.publisher = self.create_publisher(String, 'esp32/sensor_data', 10)

    def publish_sensor_data(self, data):
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f"Published: {data}")

async def handler(websocket, path, node):
    try:
        async for message in websocket:
            if isinstance(message, bytes) and len(message) == 8:  # Ensure message is 8 bytes (2 floats)
                try:
                    temperature, humidity = struct.unpack("ff", message)
                    data = f"Temperature: {temperature}, Humidity: {humidity}"
                    node.publish_sensor_data(data)
                    await websocket.send(message)  # Echo back the message
                except struct.error as e:
                    node.get_logger().error(f"Error unpacking data: {e}")
            else:
                node.get_logger().warning("Received invalid message format")
    except websockets.exceptions.ConnectionClosed:
        node.get_logger().info("WebSocket connection closed")
    except Exception as e:
        node.get_logger().error(f"Unexpected error in handler: {e}")

async def main():
    rclpy.init()
    node = WebSocketNode()

    def spin_node():
        rclpy.spin(node)

    # Run ROS 2 spinning in a separate thread
    executor = ThreadPoolExecutor()
    executor.submit(spin_node)

    try:
        server = await websockets.serve(lambda ws, p: handler(ws, p, node), "0.0.0.0", 8080)
        await server.wait_closed()
    except Exception as e:
        node.get_logger().error(f"WebSocket server error: {e}")
    finally:
        rclpy.shutdown()
        executor.shutdown()

if __name__ == "__main__":
    asyncio.run(main())