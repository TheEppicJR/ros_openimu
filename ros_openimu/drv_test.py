import sys
import time

from .aceinna.tools import OpenIMU

'''
try:
    from aceinna.tools import OpenIMU
except:  # pylint: disable=bare-except
    print('load package from local')
    sys.path.append('./src')
    from aceinna.tools import OpenIMU
'''

def simple_start():
    openimudev = OpenIMU()
    openimudev.startup()
    readback = openimudev.getdata('z1')
    print(readback)

'''
def parameters_start():
    detector = IMU(
        device_type='opemimu',
        com_port='COM3',
        baudrate=115200
    )
    detector.startup()
    readback = detector.getdata()
    print(readback)
'''

if __name__ == '__main__':
    simple_start()
    #parameters_start()
    