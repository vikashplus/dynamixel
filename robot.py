import os
import time
os.sys.path.append('../dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# os.system("sudo chmod a+rw /dev/ttyUSB0")

# Control table address
ADDR_MX_TORQUE_ENABLE       = 24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 30
ADDR_MX_PRESENT_POSITION    = 36
ADDR_MX_PRESENT_VELOCITY    = 38
ADDR_MX_MAX_VELOCITY        = 32


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
MX28                        = 1
MX64                        = 2

DXL_ID                      = MX12                          # Dynamixel ID:: (MX28:1), (MX64:2), (MX12W:4)
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyACM0".encode('utf-8')# Check which port is being used on your controller
                                                            # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 15                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed



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


    def get_pos(self):
        # Read present position
        dxl_present_position = dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_POSITION)
        if (not self.okay()):
            self.close()
            quit('error getting ADDR_MX_PRESENT_POSITION')
        return dxl_present_position


    def get_vel(self):
        # Read present position
        dxl_present_velocity = dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_PRESENT_VELOCITY)
        if (not self.okay()):
            self.close()
            quit('error getting ADDR_MX_PRESENT_VELOCITY')
        return dxl_present_velocity


    def set_des_pos(self, des_pos):
        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            print("booyeah")
            dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, DXL_ID, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay()):
                self.close()
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

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
            dxl_present_position = dy.get_pos()
            print("[ID:%03d] [cnt:%03d]  PresPos:%03d" % (DXL_ID, i, dxl_present_position))
        
        print("Zero Torque")
        dy.set_des_torque(0)
        time.sleep(1)

        print("Max CW Torque")
        dy.set_des_torque(DXL_MAX_CW_TORQUE_VALUE)
        for i in range(50):
            dxl_present_position = dy.get_pos()
            print("[ID:%03d] [cnt:%03d]  PresPos:%03d" % (DXL_ID, i, dxl_present_position))
      
        
        print("Zero Torque")
        dy.set_des_torque(0)
        time.sleep(1)


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
            dxl_present_position = dy.get_pos()
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

    # Close connection and exit
    dy.close()
