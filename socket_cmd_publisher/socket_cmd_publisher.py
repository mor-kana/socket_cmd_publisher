import socket
import struct
import json

import rclpy
from rclpy.node import Node
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
        sock.settimeout(1.0)
        while True:
            raw_len = self.recv_all(sock, 4)
            if raw_len is None:
                # クライアント切断を検出
                self.get_logger().warning("Client disconnected, forcing sticks to 0")
                # 切断後もループを継続したい場合はここで lx=ly=0 をセットして後続処理へ
                lx, ly = 0.0, 0.0
                # （必要なら他の入力も 0/False にするなど）
                # ここで変換関数を呼ぶ例：
                self.convert_joy_to_motor_pwm(lx, ly)
                # あるいは continue してもう一度ループ
                continue
            if raw_len == b'':
                # タイムアウト → 再度受信へ
                continue

            msg_len = struct.unpack('>L', raw_len)[0]
            raw = self.recv_all(sock, msg_len)
            if raw is None:
                self.get_logger().warning("Client disconnected during payload, forcing sticks to 0")
                lx, ly = 0.0, 0.0
                self.convert_joy_to_motor_pwm(lx, ly)
                continue
            if raw == b'':
                # タイムアウト → 再受信
                continue
            # --- JSON デコード ---
            try:
                payload = json.loads(raw.decode('utf-8'))
            except json.JSONDecodeError:
                self.get_logger().warning("JSON decord failed...")
                continue

            axes    = payload.get('axes', [])    # List[float]
            hats    = payload.get('hats', [])    # List[ (int,int) ]
            buttons = payload.get('buttons', []) # List[int]

            # --- 左スティック ---
            lx = axes[0] if len(axes) > 0 else 0.0
            ly = axes[1] if len(axes) > 1 else 0.0

            # --- 右スティック ---
            rx = axes[2] if len(axes) > 2 else 0.0
            ry = axes[3] if len(axes) > 3 else 0.0

            # --- トリガー（LT, RT）---
            lt = axes[4] if len(axes) > 4 else 0.0
            rt = axes[5] if len(axes) > 5 else 0.0

            # --- 十字キー（D-Pad）---
            dpad_x, dpad_y = (hats[0] if len(hats) > 0 else (0,0))

            # --- ボタン ---
            # F310 の DirectInput モード想定: インデックス順に名前付け
            button_names = [
                'A','B','X','Y',     # 0-3
                'LB','RB',           # 4-5
                'Back','Start',      # 6-7
                'LStick','RStick'    # 8-9
            ]
            btn_state = {}
            for idx, name in enumerate(button_names):
                btn_state[name] = (buttons[idx] == 1) if idx < len(buttons) else False

            # --- ログ出力（例） ---
            self.get_logger().info(
                f"LeftStick  ({lx:.2f}, {ly:.2f}), "
                f"RightStick ({rx:.2f}, {ry:.2f}), "
                f"LT={lt:.2f}, RT={rt:.2f}"
            )
            self.get_logger().info(f"D-Pad      ({dpad_x}, {dpad_y})")
            self.get_logger().info(
                "Buttons: " +
                ", ".join(f"{name}={state}"
                          for name, state in btn_state.items())
            )
            self.convert_joy_to_motor_pwm(lx, ly)
        # while を抜けたらここに来る
        sock.close()
        self.get_logger().info("handle_client exited cleanly")


    def recv_all(self, sock, size):
        buf = b''
        while len(buf) < size:
            chunk = sock.recv(size - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def convert_joy_to_motor_pwm(self, axis_x, axis_y):
        # 入力補正: 上 = 前進, 右 = 右旋回
            v = axis_y * V_MAX
            omega = axis_x * OMEGA_MAX

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
