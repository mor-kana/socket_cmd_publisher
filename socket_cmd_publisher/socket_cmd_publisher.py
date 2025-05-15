import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8

# --- 設定 ---
HOST = '0.0.0.0'            # 全インターフェイスで待ち受け
PORT = 5000                 # ポート番号
ALLOWED_IP = '100.113.235.4'  # 許可するクライアントの IP

class SocketCmdPublisher(Node):
    def __init__(self):
        super().__init__('socket_cmd_publisher')
        self.pub_ = self.create_publisher(UInt8, 'cmd_code', 10)
        # ソケット初期化
        self.srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_sock.bind((HOST, PORT))
        self.srv_sock.listen(1)
        self.get_logger().info(f'Listening on {HOST}:{PORT}')
        self.accept_loop()

    def accept_loop(self):
        while rclpy.ok():
            conn, addr = self.srv_sock.accept()
            client_ip, client_port = addr
            if client_ip != ALLOWED_IP:
                self.get_logger().warn(f'Rejecting unauthorized IP: {client_ip}')
                conn.close()
                continue

            self.get_logger().info(f'Accepted connection from {client_ip}:{client_port}')
            self.handle_client(conn)
            conn.close()
            self.get_logger().info(f'Connection closed: {client_ip}')

    def handle_client(self, sock):
        while True:
                # 8バイト受信
            raw = self.recv_all(sock, 8)
            if raw is None:
                break

            axis_x, axis_y = struct.unpack('>ff', raw)
            # X 値を publish
            msg = float()
            msg.data = axis_x
            self.pub_.publish(msg)
            self.get_logger().info(f'Published cmd_code: {axis_x}')
            # 続いて Y を受け取って捨てる
            if self.recv_all(sock, 4) is None:
                break

    def recv_all(self, sock, size):
        buf = b''
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

def main(args=None):
    rclpy.init(args=args)
    node = SocketCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
