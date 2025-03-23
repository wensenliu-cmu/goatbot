import sys
import time
import signal
import threading
import common.ros_robot_controller_sdk as rrc

print('''
**********************************************************
********功能:幻尔科技树莓派扩展板，控制总线舵机转动**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！
----------------------------------------------------------
''')
board = rrc.Board()
board.enable_reception()
start = True

# 关闭前处理
def Stop(signum, frame):
    global start
    start = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

# Run IMU Calibration
# Collect 10 seconds of data then average out to fit "zeros"

def bus_servo_test(board):
    #servo_id = board.bus_servo_read_id() # can only read servo id when there is only one servo connected
    #servo_id = servo_id[0]
    servos = [1, 2]
    for servo_id in servos:
        vin =  board.bus_servo_read_vin(servo_id)
        temp = board.bus_servo_read_temp(servo_id)
        position = board.bus_servo_read_position(servo_id)
        # 输出舵机状态
        print("id:", servo_id)
        print('vin:', vin)
        print('temp:',temp)
        print('position',position)

if __name__ == '__main__':
    try:
        #board.bus_servo_set_id(servo_id_now=1, servo_id_new=2)
        board.bus_servo_set_position(4, [[1, 0], [2, 0]])
        time.sleep(5)
        board.bus_servo_set_position(0.05, [[1, 60], [2, 60]]) # every 50ms we can turn max 16.67 degrees or 62.5 for the serial commands # make sure control clamps to this
        time.sleep(1)
        while start:
            bus_servo_test(board)
            time.sleep(1)
    except KeyboardInterrupt:
        print(f"Keyboard Interrupt")