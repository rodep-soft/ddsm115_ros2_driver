#!/usr/bin/env python3
import sys
import serial
import time
import rclpy
from rclpy.node import Node

def calculate_crc8(data):
    """
    CRC-8/MAXIM (Polynomial: 0x31 / x8 + x5 + x4 + 1)
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def set_motor_id(port, new_id, logger=None):
    try:
        ser = serial.Serial(port, 115200, timeout=0.5)
        command = bytearray([0xAA, 0x55, 0x53, new_id, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        
        msg = f"[{port}] IDを {new_id} に設定中..."
        if logger: logger.info(msg)
        else: print(msg)
        
        for i in range(5):
            ser.write(command)
            time.sleep(0.1)
            if logger: logger.info(f"送信 {i+1}/5")
            else: print(f"送信 {i+1}/5")
            
        success_msg = "設定完了。モータを再起動して反映を確認してください。"
        if logger: logger.info(success_msg)
        else: print(success_msg)
        
        ser.close()
        
    except Exception as e:
        err_msg = f"エラーが発生しました: {e}"
        if logger: logger.error(err_msg)
        else: print(err_msg)

class MotorIdSetterNode(Node):
    def __init__(self):
        super().__init__('set_motor_id_node')
        
        # ROS 2パラメータの宣言とデフォルト値の設定
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('id', 1)
        
        # パラメータの取得
        port = self.get_parameter('port').value
        new_id = self.get_parameter('id').value
        
        # バリデーション
        if not (1 <= new_id <= 253):
            self.get_logger().error("IDは1から253の間で指定してください。")
            sys.exit(1)
            
        # 実際の通信処理の呼び出し
        set_motor_id(port, new_id, self.get_logger())

def main(args=None):
    rclpy.init(args=args)
    
    # ノードを初期化して処理を実行
    node = MotorIdSetterNode()
    
    # 継続してスピンさせる必要はないので、処理が終わったら破棄して終了
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()