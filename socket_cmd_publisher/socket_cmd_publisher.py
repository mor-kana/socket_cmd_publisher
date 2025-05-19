import socket
import struct

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from geometry_msgs.msg import Vector3

# 設定
HOST = '0.0.0.0'            # 全インターフェイスで待ち受け
PORT = 5000                 # ポート番号
ALLOWED_IP = '100.113.235.4'  # 許可するクライアントの IP

# モータ出力設定
MAX_PWM = 255
V_MAX = 1.0
OMEGA_MAX = 1.0

class SocketCmdPublisher(Node):
    def __init__(self):
        super().__init__('socket_cmd_publisher')

        # トピックのPublisher
        self.pwm_pub = self.create_publisher(Vector3, 'motor_pwm', 10)

        # ソケット初期化
        self.srv_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv_sock.bind((HOST, PORT))
        self.srv_sock.listen(1)
        self.get_logger().info(f'Listening on {HOST}:{PORT}')

        # 接続ループ開始
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
            raw = self.recv_all(sock, 8)
            if raw is None:
                break

            # 受信: axis_x, axis_y（float32, ビッグエンディアン）
            axis_x, axis_y = struct.unpack('>ff', raw)
            self.convert_joy_to_motor_pwm(axis_x, axis_y)


    def recv_all(self, sock, size):
        buf = b''
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def convert_joy_to_motor_pwm(self, axis_x, axis_y):
        # 入力補正: 上 = 前進, 右 = 右旋回（負符号を考慮）
            v = -axis_y * V_MAX
            omega = -axis_x * OMEGA_MAX

            # 差動駆動モデルによる左右速度計算
            left_speed = v - omega
            right_speed = v + omega

            # PWM変換
            scale = MAX_PWM / (V_MAX + OMEGA_MAX)
            left_pwm = int(max(-MAX_PWM, min(MAX_PWM, left_speed * scale)))
            right_pwm = int(max(-MAX_PWM, min(MAX_PWM, right_speed * scale)))

            # ログ出力
            self.get_logger().info(f"axis_x={axis_x:.2f}, axis_y={axis_y:.2f} -> L={left_pwm}, R={right_pwm}")

            # Vector3メッセージとしてPublish
            pwm_msg = Vector3()
            pwm_msg.x = float(left_pwm)  # 左モータ出力
            pwm_msg.y = float(right_pwm) # 右モータ出力
            pwm_msg.z = 0.0              # 未使用
            self.pwm_pub.publish(pwm_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = SocketCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
