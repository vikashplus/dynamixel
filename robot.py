import os
import time
os.sys.path.append('../dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

from api import allegro as al
import numpy as np
import time

# os.system("sudo chmod a+rw /dev/ttyUSB0")
##

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_VELOCITY    = 38
ADDR_MX_MAX_VELOCITY        = 32
ADDR_MX_CW_ANGLE_LIMIT      = 6
ADDR_MX_CCW_ANGLE_LIMIT     = 8
ADDR_MX_MAX_TORQUE          = 14
ADDR_MX_RESOLUTION          = 22
ADDR_MX_OFFSET              = 20


# torque control mode options
ADDR_MX_TORQUE_CONTROL_MODE = 70
ADDR_MX_GOAL_TORQUE         = 71                            # Lowest byte of goal torque value
DXL_NULL_TORQUE_VALUE       = 0
DXL_MIN_CW_TORQUE_VALUE     = 1024
DXL_MAX_CW_TORQUE_VALUE     = 2047
DXL_MIN_CCW_TORQUE_VALUE    = 0
DXL_MAX_CCW_TORQUE_VALUE    = 1023

# Protocol version
PROTOCOL_VERSION            = 1                             # See which protocol version is used in the Dynamixel

# Default setting
MX12                        = 1
MX28                        = 4
MX64                        = 2

DXL_ID                      = MX12                          # Dynamixel ID:: (MX28:1), (MX64:2), (MX12W:4)
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 15                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

# Flags
MULTI_TURN                  = 0

# Limits
CW_ANGLE_LIMIT              = 0

# Unit conversions from raw to SI
RESOLUTION_DIVIDER          = 4
POS_RESOLUTION              = 0.088
MAX_RAW_POS_EMP             = 262128
MAX_DEG_POS_EMP             = int(MAX_RAW_POS_EMP * POS_RESOLUTION)
MX_MAX_GOAL_POS             = 28672
MX_MIN_GOAL_POS             = -28672




class robot():
    def __init__(self):
        # default mode 
        self.ctrl_mode = TORQUE_DISABLE

        # Initialize PortHandler Structs
        # Set the port path and Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(DEVICENAME)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            quit("Failed to open the port!")

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            quit("Failed to change the baudrate!")

        # Enable Dynamixel Torque
        self.engage_motor(True)

        print("Dynamixel has been successfully connected")

        dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_RESOLUTION, RESOLUTION_DIVIDER)


    def okay(self):
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(PROTOCOL_VERSION, dxl_comm_result))
            return False
        elif dxl_error != 0:
            print(dynamixel.getRxPacketError(PROTOCOL_VERSION, dxl_error))
            return False
        else:
            return True


    # Engage/ Disengae the motors. enable = True/ False
    def engage_motor(self, enable):
        dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_ENABLE, enable)
        if (not self.okay()):
            quit('error with ADDR_MX_TORQUE_ENABLE')


    def get_pos(self, deg = False):
        # Read present position
        dxl_present_position = dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_POSITION) 
        if MULTI_TURN:
            dxl_present_position *= RESOLUTION_DIVIDER
        if (not self.okay()):
            self.close()
            quit('error getting ADDR_MX_PRESENT_POSITION')

        # convert to degrees
        if deg:
            
            dxl_present_position = int(POS_RESOLUTION * dxl_present_position)

        return dxl_present_position


    def get_vel(self, raw = False):
        # Read present position
        dxl_present_velocity = dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_VELOCITY)
        if (not self.okay()):
            self.close()
            quit('error getting ADDR_MX_PRESENT_VELOCITY')
        return dxl_present_velocity


    def set_des_pos(self, des_pos, deg = False):
        # convert from degrees to raw
        # deg: center = 180, max = 360, min = 0

        if deg:
            des_pos = int(des_pos / POS_RESOLUTION)


        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            print("booyeah")
            dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay()):
                self.close()
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

        # enforce limits
        if MULTI_TURN:
            cw_angle_limit = 0 #dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CW_ANGLE_LIMIT)
            ccw_angle_limit = MAX_RAW_POS_EMP #dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CCW_ANGLE_LIMIT)
        else:
            cw_angle_limit = dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CW_ANGLE_LIMIT)
            ccw_angle_limit = dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_CCW_ANGLE_LIMIT)

        #print(cw_angle_limit)
        
        if des_pos < cw_angle_limit:
            des_pos = cw_angle_limit
            print("lower position bound reached")
        elif des_pos > ccw_angle_limit:
            des_pos = ccw_angle_limit
            print("upper position bound reached")


        # Write goal position
        dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_POSITION, des_pos)
        if (not self.okay()):
            self.close()
            quit('error setting ADDR_MX_GOAL_POSITION')


    def set_des_torque(self, des_tor):
        # If in position mode, activate torque mode
        if(self.ctrl_mode == TORQUE_DISABLE):
            dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_ENABLE)
            if (not self.okay()):
                self.close()
                quit('error enabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_ENABLE

        # Write goal position
        dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_GOAL_TORQUE, des_tor)
        if (not self.okay()):
            self.close()
            quit('error setting ADDR_MX_GOAL_TORQUE')


    def set_max_vel(self, max_vel):
        dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_MAX_VELOCITY, max_vel)
        if (not self.okay()):
            self.close()
            quit('error setting ADDR_MX_MAX_VELOCITY')


    def close(self):
        # Disengage Dynamixels
        self.engage_motor(False)

        # Close port
        dynamixel.closePort(self.port_num)



if __name__ == '__main__':
    
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
    index = 0
    
    dy = robot()
    host = "128.208.4.243"
    al.hx_connect(host)
    #info = al.hx_robot_info()
    u = np.ones((16,))
    
    
    
    
    """

    if (DXL_ID == MX12):
        dy.engage_motor(False)
        input("Motor disengaged. Press enter and apply external forces")
        for i in range(100):
            dxl_present_position = dy.get_pos()
            dxl_present_velocity = dy.get_vel()
            print("[ID:%03d] [cnt:%03d]  Pos:%03d, Vel:%03d" % (DXL_ID, i, dxl_present_position, dxl_present_velocity))
        dy.engage_motor(True)
        dy.set_max_vel(60)
        print("Motor engaged")

    
    # Test torque mode =============================
    if(DXL_ID == MX64):
        print("Zero Torque")
        dy.set_des_torque(0)
        time.sleep(1)
        

        print("Max CCW Torque")
        dy.set_des_torque(DXL_MAX_CCW_TORQUE_VALUE)
        for i in range(50):
            dxl_present_position = dy.get_pos(True)
            print("[ID:%03d] [cnt:%03d]  PresPos:%03d" % (DXL_ID, i, dxl_present_position))
        
        print("Zero Torque")
        dy.set_des_torque(0)
        time.sleep(1)

        print("Max CW Torque")
        dy.set_des_torque(DXL_MAX_CW_TORQUE_VALUE)
        for i in range(50):
            dxl_present_position = dy.get_pos(True)
            print("[ID:%03d] [cnt:%03d]  PresPos:%03d" % (DXL_ID, i, dxl_present_position))
      
        
        print("Zero Torque")
        dy.set_des_torque(0)
        time.sleep(1)

    """
    """
    # Test position mode =============================
    while 1:
        cnt = 0
        # wait for user input
        user = input("Press ENTER to continue! (or press q+ENTER to quit!)")
        if user == 'q':
            break

        # set goal position
        dy.set_des_pos(dxl_goal_position[index])


        
        # wait and read sensors
        while 1:
            dxl_present_position = dy.get_pos(True)
            dxl_present_velocity = dy.get_vel()
            print("[ID:%03d] [cnt:%03d] GoalPos:%03d, Pos:%03d, Vel:%03d" % (DXL_ID, i,  dxl_goal_position[index], dxl_present_position, dxl_present_velocity))
            cnt = cnt + 1
            if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                break
        
        # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0
    """
    # Resetting
    allegro_open_hand = [-0.19234973192214966, 0.6015477776527405, 0.003994341474026442, 0.23034034669399261, -0.0756261944770813, 0.3661479651927948, -0.08956200629472733, 0.551929235458374, 0.0759812518954277, 0.4423067271709442, -0.030623283237218857, -2.9456934928894043, 0.8270061612129211, 0.8169759511947632, 0.3208787441253662, -0.24019305408000946]

    POS_THRESHOLD = 1                       # degrees
    RESET_POS = 180         # degrees

    dy.set_max_vel(100)
    al.hx_ctrl(allegro_open_hand, False)
    time.sleep(1)
    i = 0
    curr_pos = dy.get_pos(deg = True)
    while abs(curr_pos - RESET_POS) > POS_THRESHOLD:
        i += 1
        if i % 50000 == 0:
            al.hx_ctrl(allegro_open_hand, False)
            dy.set_des_pos(RESET_POS, deg = True)
            curr_pos = dy.get_pos(deg = True)
            print("GOAL POS: %f, CURR POSITION: %f" %(RESET_POS / POS_RESOLUTION, curr_pos))
    al.hx_ctrl(allegro_open_hand, False)
    print("Made it to the goal! GOAL POS: %f, CURR POSITION: %f" %(RESET_POS, dy.get_pos(deg = True)))

    

    
    #allegro_reset_pos = [0.2711713910102844, 1.3294055461883545, 0.4237552285194397, 0.9109761118888855, -0.21374164521694183, 0.2763196527957916, 1.2242212295532227, 0.5426090955734253, -0.36357381939888, 0.822479248046875, 1.3656209707260132, 1.132617712020874, 0.6276441812515259, 0.7996671199798584, 0.45047295093536377, 1.3058832883834839]
    #al.hx_ctrl(allegro_reset_pos, False);
    
    """
    TIME_THRESHOLD = 10
    t0 = time.time()
    

    print(dynamixel.read2ByteTxRx(dy.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_OFFSET))
    

    dy.set_max_vel(100)
    while time.time() - t0 < TIME_THRESHOLD:
        
        i += 1
        if i % 20000 == 0:
            #dy.set_des_pos(RESET_POS, deg = True)
            print("GOAL POS: %f, CURR POSITION: %f" %(RESET_POS / POS_RESOLUTION, dy.get_pos(deg = False)))
    """

    # Close connection and exit
    dy.close()
