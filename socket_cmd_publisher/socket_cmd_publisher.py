import socket
import struct
import json

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

# 設定
HOST = '0.0.0.0'             # 全インターフェイスで待ち受け
PORT = 5000                  # ポート番号
# ALLOWED_IP = '100.113.235.4' # 許可するクライアントの IP (NGSH)
ALLOWED_IP = '100.118.30.28' # 許可するクライアントの IP (MRKN)
# ALLOWED_IP = '100.84.44.94'  # 許可するクライアントの IP (SNSA)
print("Allowed ip:", ALLOWED_IP)
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
            print(f'client ip ={client_ip}')
            if client_ip != ALLOWED_IP:
                self.get_logger().warn(f'Rejecting unauthorized IP: {client_ip}')
                conn.close()
                continue

            self.get_logger().info(f'Accepted connection from {client_ip}:{client_port}')
            self.handle_client(conn)
            conn.close()
            self.get_logger().info(f'Connection closed: {client_ip}')

    def handle_client(self, sock):
        # タイムアウトを設定（recv が長時間ブロックしないように）
        sock.settimeout(1.0)

        while True:
            # --- ヘッダー受信（4バイト）---
            raw_len = self.recv_all(sock, 4)
            if raw_len is None:
                # クライアント切断を検出 → モーター停止してループ終了
                self.get_logger().warning("Client disconnected, stopping motors")
                self.convert_joy_to_motor_pwm(0.0, 0.0)
                break
            if raw_len == b'':
                # タイムアウト → 再試行
                continue

            msg_len = struct.unpack('>L', raw_len)[0]

            # --- 本体受信 ---
            raw = self.recv_all(sock, msg_len)
            if raw is None:
                # 切断検出
                self.get_logger().warning("Client disconnected during payload, stopping motors")
                self.convert_joy_to_motor_pwm(0.0, 0.0)
                break
            if raw == b'':
                # タイムアウト → 再試行
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

            # ログ出力
            self.get_logger().info(
                f"LStick  ({lx:.2f}, {ly:.2f}), "
                f"RStick ({rx:.2f}, {ry:.2f}), "
                f"LT={lt:.2f}, RT={rt:.2f}"
            )
            self.get_logger().info(f"D-Pad({dpad_x}, {dpad_y})")
            self.get_logger().info("Buttons: " +", ".join(f"{name}={'P' if state else 'R'}"for name, state in btn_state.items())
            )

            # モーター制御関数に渡す
            self.convert_joy_to_motor_pwm(lx, ly)

        # ループを抜けたらソケットを閉じ、ノードへ制御を戻す
        sock.close()
        self.get_logger().info("handle_client exiting, motors stopped")

    def recv_all(self, sock, size):
        buf = b''
        while len(buf) < size:
            try:
                chunk = sock.recv(size - len(buf))
            except socket.timeout:
                # タイムアウト時は空バイトを返し、呼び出し元で再試行
                return b''
            if not chunk:
                # 切断時は None を返す
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
        pwm_msg.x = float(left_pwm)
        pwm_msg.y = float(right_pwm)
        pwm_msg.z = 0.0
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

