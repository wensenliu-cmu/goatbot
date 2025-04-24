import common.board_helper_functions as bhf
import time

if __name__ == '__main__':
    helper = bhf.BoardHelper()
    try:
        while True:
            IMU_reading = helper.read_IMU()
            out_str = f""
            if IMU_reading is not None:
                for key in IMU_reading:
                    out_str += f"{key+':':<6}{IMU_reading[key]:.3f}\t"
                print(out_str, end="\r")
                time.sleep(0.05)
    except Exception as e:
        print(f"Error: {e}")