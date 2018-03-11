import os
import time
os.sys.path.append('../dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library


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
MX28                        = 2
MX64                        = 3

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
    def __init__(self, motor_id):

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
            print("Failed to open the port")
            os.system("sudo chmod a+rw /dev/ttyACM0")
            print("Editing permissions and trying again")
            if dynamixel.openPort(self.port_num):
                print("Succeeded to open the port!")
            else:
                quit("Failed to open the port! Run following command and try again.\nsudo chmod a+rw /dev/ttyACM0")

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            quit("Failed to change the baudrate!")

        # Enable Dynamixel Torque
        self.engage_motor(motor_id, True)

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
    def engage_motor(self, motor_id, enable):
        for dxl_id in motor_id:
            dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_ENABLE, enable)
        if (not self.okay()):
            quit('error with ADDR_MX_TORQUE_ENABLE')


    def get_pos(self, motor_id):
        dxl_present_position = []
        # Read present position
        for dxl_id in motor_id:
            dxl_present_position.append(dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_POSITION))
        if (not self.okay()):
            self.close(motor_id)
            quit('error getting ADDR_MX_PRESENT_POSITION')
        return dxl_present_position


    def get_vel(self, motor_id):
        dxl_present_velocity = []
        # Read present position
        for dxl_id in motor_id:
            dxl_present_velocity.append(dynamixel.read2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_VELOCITY))
        if (not self.okay()):
            self.close(motor_id)
            quit('error getting ADDR_MX_PRESENT_VELOCITY')
        return dxl_present_velocity


    def set_des_pos(self, motor_id, des_pos):
        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay()):
                self.close(motor_id)
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

        # Write goal position
        for dxl_id in motor_id:
            dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_GOAL_POSITION, des_pos)
        if (not self.okay()):
            self.close(motor_id)
            quit('error setting ADDR_MX_GOAL_POSITION')


    def set_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        if(self.ctrl_mode == TORQUE_DISABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_ENABLE)
            if (not self.okay()):
                self.close(motor_id)
                quit('error enabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_ENABLE

        # Write goal position
        for dxl_id in motor_id:
            dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_GOAL_TORQUE, des_tor)
        if (not self.okay()):
            self.close(motor_id)
            quit('error setting ADDR_MX_GOAL_TORQUE')


    def set_max_vel(self, motor_id, max_vel):
        for dxl_id in motor_id:
            dynamixel.write2ByteTxRx(self.port_num, PROTOCOL_VERSION, dxl_id, ADDR_MX_MAX_VELOCITY, max_vel)
        if (not self.okay()):
            self.close(motor_id)
            quit('error setting ADDR_MX_MAX_VELOCITY')


    def close(self, motor_id):
        # Disengage Dynamixels
        self.engage_motor(motor_id, False)

        # Close port
        dynamixel.closePort(self.port_num)



if __name__ == '__main__':
    
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position
    index = 0
    
    dxl_ids =  [MX12, MX28]
    dy = robot(dxl_ids)

    dy.engage_motor(dxl_ids, False)
    input("Motor disengaged. Press enter and apply external forces")
    for i in range(100):
        dxl_present_position = dy.get_pos(dxl_ids)
        dxl_present_velocity = dy.get_vel(dxl_ids)
        for j in range(len(dxl_ids)):
            print("cnt:%03d, dxl_id:%01d ==> Pos:%04d, Vel:%04d" % (i, dxl_ids[j], dxl_present_position[j], dxl_present_velocity[j]))
    dy.engage_motor(dxl_ids, True)
    dy.set_max_vel(dxl_ids, 60)
    print("Motor engaged")

    # Test torque mode =============================
    if(0): #only for MX64s
        print("Zero Torque")
        dy.set_des_torque(dxl_ids, 0)
        time.sleep(1)
        

        print("Max CCW Torque")
        dy.set_des_torque(dxl_ids, DXL_MAX_CCW_TORQUE_VALUE)
        for i in range(50):
            dxl_present_position = dy.get_pos(dxl_ids)
            for j in range(len(dxl_ids)):
                print("cnt:%03d, dxl_id:%01d ==> Pos:%04d, Vel:%04d" % (i, dxl_ids[j], dxl_present_position[j], dxl_present_velocity[j]))
        
        print("Zero Torque")
        dy.set_des_torque(dxl_ids, 0)
        time.sleep(1)

        print("Max CW Torque")
        dy.set_des_torque(dxl_ids, DXL_MAX_CW_TORQUE_VALUE)
        for i in range(50):
            dxl_present_position = dy.get_pos(dxl_ids)
            for j in range(len(dxl_ids)):
                print("cnt:%03d, dxl_id:%01d ==> Pos:%04d, Vel:%04d" % (i, dxl_ids[j], dxl_present_position[j], dxl_present_velocity[j]))
      
        
        print("Zero Torque")
        dy.set_des_torque(dxl_ids, 0)
        time.sleep(1)


    # Test position mode =============================
    while 1:
        cnt = 0
        # wait for user input
        user = input("Press ENTER to continue! (or press q+ENTER to quit!)")
        if user == 'q':
            break

        # set goal position
        dy.set_des_pos(dxl_ids, dxl_goal_position[index])


        # wait and read sensors
        while 1:
            dxl_present_position = dy.get_pos(dxl_ids)
            dxl_present_velocity = dy.get_vel(dxl_ids)
            for j in range(len(dxl_ids)):
                print("cnt:%03d, dxl_id:%01d ==> GoalPos:%04d, Pos:%04d, Vel:%04d" % (i, dxl_ids[j], dxl_goal_position[index], dxl_present_position[j], dxl_present_velocity[j]))

            cnt = cnt + 1
            if not (abs(dxl_goal_position[index] - dxl_present_position[0]) > DXL_MOVING_STATUS_THRESHOLD): # checking only the first motor
                break

        # Change goal position
        if index == 0:
            index = 1
        else:
            index = 0

    # Close connection and exit
    dy.close(dxl_ids)
    print('successful exit')
