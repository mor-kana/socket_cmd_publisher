import socket
import struct
import json

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Joyメッセージを使用

# 設定
HOST = '0.0.0.0'             # 全インターフェイスで待ち受け
PORT = 5000                  # ポート番号
ALLOWED_IP = '100.113.235.4' # 許可するクライアントの IP

class SocketCmdPublisher(Node):
    def __init__(self):
        super().__init__('socket_cmd_publisher')

        # JoyトピックのPublisher
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

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
        # タイムアウトを設定（recv が長時間ブロックしないように）
        sock.settimeout(1.0)

        while True:
            # --- ヘッダー受信（4バイト）---
            raw_len = self.recv_all(sock, 4)
            if raw_len is None:
                # クライアント切断を検出 → ループ終了
                self.get_logger().warning("Client disconnected")
                break
            if raw_len == b'':
                # タイムアウト → 再試行
                continue

            msg_len = struct.unpack('>L', raw_len)[0]

            # --- 本体受信 ---
            raw = self.recv_all(sock, msg_len)
            if raw is None:
                # 切断検出
                self.get_logger().warning("Client disconnected during payload")
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

            # Joyメッセージとしてパブリッシュ
            self.publish_joy_message(axes, hats, buttons)

        # ループを抜けたらソケットを閉じ、ノードへ制御を戻す
        sock.close()
        self.get_logger().info("handle_client exiting")

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

    def publish_joy_message(self, axes, hats, buttons):
        # Joyメッセージ構築
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()

        # axes: [LX, LY, RX, RY, LT, RT] + D-Pad
        joy_msg.axes = [0.0] * 8
        for i in range(min(6, len(axes))):
            joy_msg.axes[i] = float(axes[i])

        if len(hats) > 0:
            joy_msg.axes[6] = float(hats[0][0])  # D-Pad X
            joy_msg.axes[7] = float(hats[0][1])  # D-Pad Y

        joy_msg.buttons = [int(val) for val in buttons[:11]]  # 最大11ボタン（A〜RStickまで）

        self.joy_pub.publish(joy_msg)
        self.get_logger().info(f"Joy message published: axes={joy_msg.axes}, buttons={joy_msg.buttons}")

def main(args=None):
    rclpy.init(args=args)
    node = SocketCmdPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
