# Control Loop Modes (定数定義)
uint8 MODE_CURRENT = 1  # 電流(トルク)制御
uint8 MODE_VELOCITY = 2 # 速度制御
uint8 MODE_POSITION = 3 # 位置制御

# Brake Mode (停止時の挙動)
uint8 BRAKE_RELEASE = 0 # フリー
uint8 BRAKE_LOCK = 1    # ロック

# Message Fields
uint8 mode
float64 value
uint8 brake_mode

# MODE_CURRENT: value = 電流 (A) -8.0~8.0(A)
# MODE_VELOCITY: value = 速度 (RPM) -330~330(RPM)
# MODE_POSITION: value = 位置 (回転数) 0~360(°)