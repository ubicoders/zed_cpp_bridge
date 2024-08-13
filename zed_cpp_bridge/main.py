import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import time
import flatbuffers
from .aruco_msgs_generated import StereoImageMarkers, ImageMarkers, Marker, Point2D
from .msg_decoder import MarkerDecoder
import json
class UDPClientPublisher(Node):

    def __init__(self):
        super().__init__('udp_client_publisher')
        self.publisher_ = self.create_publisher(String, 'example_topic', 10)
        self.get_logger().info("UDPClientPublisher node has been started.")

        # Create a UDP socket
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = ('127.0.0.1', 12345)  # Server IP and port
        self.get_logger().info(f"UDP client started and sending to {self.server_address}.")

        # Create a timer to send data periodically
        self.timer = self.create_timer(0.001, self.udp_client_callback)

        # Initialize the MarkerDecoder
        self.marker_decoder = MarkerDecoder()

    def udp_client_callback(self):
        start_time = time.time()  # Record the start time

        # Simulate sending data
        message = "Hello, UDP Server"
        self.udp_socket.sendto(message.encode('utf-8'), self.server_address)

        # Receive a response from the server
        data, addr = self.udp_socket.recvfrom(1024)  # Buffer size is 1024 bytes
        buf = bytearray(data)

        try:
            # Decode the buffer and log the marker information
            markers_info = self.marker_decoder.decode(buf)
            self.get_logger().info(f"Left Image Markers: {json.dumps(markers_info['left'], indent=4)}")
            self.get_logger().info(f"Right Image Markers: {json.dumps(markers_info['right'], indent=4)}")
        except ValueError as e:
            self.get_logger().error(str(e))

        # self.get_logger().info(f"Received: {data.decode('utf-8')} from {addr}")

        # Publish the received message
        # self.publish_message(data.decode('utf-8'))

        end_time = time.time()  # Record the end time
        elapsed_time = end_time - start_time  # Calculate the elapsed time

        self.get_logger().info(f"Callback execution time: {elapsed_time:.4f} seconds. Frequency: {1.0 / elapsed_time:.2f} Hz")

    def publish_message(self, message):
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = UDPClientPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
