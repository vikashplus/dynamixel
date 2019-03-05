import os, ctypes
import time
import numpy as np
os.sys.path.append('../dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses Dynamixel SDK library

# Control table address
ADDR_MX_TORQUE_ENABLE       = 64#24                            # Control table address is different in Dynamixel model
ADDR_MX_GOAL_POSITION       = 116#30
ADDR_MX_PRESENT_POSITION    = 132#36
ADDR_MX_PRESENT_VELOCITY    = 128#38
ADDR_MX_PRESENT_POS_VEL     = 128#36                            # Trick to get position and velocity at once
ADDR_MX_MAX_VELOCITY        = 44
ADDR_MX_HARDWARE_ERROR      = 70

# Data Byte Length
LEN_MX_PRESENT_POSITION     = 4
LEN_MX_PRESENT_VELOCITY     = 4
LEN_MX_PRESENT_POS_VEL      = 8
LEN_MX_GOAL_POSITION        = 4


# torque control mode options (left over from P1)
ADDR_MX_TORQUE_CONTROL_MODE = 70
ADDR_MX_GOAL_TORQUE         = 71                            # Lowest byte of goal torque value
DXL_NULL_TORQUE_VALUE       = 0
DXL_MIN_CW_TORQUE_VALUE     = 1024
DXL_MAX_CW_TORQUE_VALUE     = 2047
DXL_MIN_CCW_TORQUE_VALUE    = 0
DXL_MAX_CCW_TORQUE_VALUE    = 1023
LEN_MX_GOAL_TORQUE          = 2


# Default setting
MX12                        = 1
MX28                        = 2
MX64                        = 3
XH430                       = 1

# Settings for MX28
POS_SCALE = 2*np.pi/4096 #(=.088 degrees)
VEL_SCALE = 0.11 * 2 * np.pi / 60 #(=0.11rpm)


TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 100                           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000                          # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 15                            # Dynamixel moving status threshold

ESC_ASCII_VALUE             = 0x1b

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed



class dxl():

    # DEVICENAME: Port name being used on your controller # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
    def __init__(self, motor_id, BAUDRATE=1000000, DEVICENAME="/dev/ttyACM0", PROTOCOL_VERSION=1):

        self.n_motors = len(motor_id)
        self.PROTOCOL_VERSION = PROTOCOL_VERSION

        # default mode 
        self.ctrl_mode = TORQUE_DISABLE

        # Initialize PortHandler Structs
        # Set the port path and Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(DEVICENAME.encode('utf-8'))

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port")
            os.system("sudo chmod a+rw %s"%DEVICENAME)
            print("Editing permissions and trying again")
            if dynamixel.openPort(self.port_num):
                print("Succeeded to open the port!")
            else:
                quit("Failed to open the port! Run following command and try again.\nsudo chmod a+rw %s"%DEVICENAME)

        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            quit("Failed to change the baudrate!")

        # Enable Dynamixel Torque
        self.engage_motor(motor_id, True)

        # Initialize Group instance
        
        # controls
        self.group_desPos = dynamixel.groupSyncWrite(self.port_num, self.PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)
        self.group_desTor = dynamixel.groupSyncWrite(self.port_num, self.PROTOCOL_VERSION, ADDR_MX_GOAL_TORQUE, LEN_MX_GOAL_TORQUE)

        # positions
        self.group_pos = dynamixel.groupBulkRead(self.port_num, self.PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_pos, m_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_posfailed" % (m_id))
                quit()
        
        # velocities
        self.group_vel = dynamixel.groupBulkRead(self.port_num, self.PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_vel, m_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_vel failed" % (m_id))
                quit()

        # positions and velocities
        self.group_pos_vel = dynamixel.groupBulkRead(self.port_num, self.PROTOCOL_VERSION)
        for m_id in motor_id:
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(self.group_pos_vel, m_id, ADDR_MX_PRESENT_POS_VEL, LEN_MX_PRESENT_POS_VEL)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupBulkRead addparam_posfailed" % (m_id))
                quit()

        # buffers
        self.dxl_present_position = float('nan')*np.zeros(self.n_motors)
        self.dxl_present_velocity = float('nan')*np.zeros(self.n_motors)
        self.dxl_last_position = float('nan')*np.zeros(self.n_motors)
        self.dxl_last_velocity = float('nan')*np.zeros(self.n_motors)
        print("Dynamixels successfully connected")


    # Cheak health
    def okay(self, motor_id):
        dxl_comm_result = dynamixel.getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
        dxl_error = dynamixel.getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)

        if(dxl_comm_result != COMM_SUCCESS) or (dxl_error != COMM_SUCCESS):
            # Print apprpriate error
            if dxl_comm_result != COMM_SUCCESS:
                print("\n" + str(dynamixel.getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result)))
            if dxl_error != COMM_SUCCESS:
                print("\n" + str(dynamixel.getRxPacketError(self.PROTOCOL_VERSION, dxl_error)) + ". Error_id:" + str(dxl_error))
            
            # print hardware status 
            print("Motor id(hardware status): [ ", end='')
            for m_id in motor_id:
                print("%d(%d), " % (m_id, dynamixel.read1ByteTxRx(self.port_num, self.PROTOCOL_VERSION,\
                     m_id, ADDR_MX_HARDWARE_ERROR)), end='')
            print("]", flush=True)
            return False
        else:
            return True


    # Engage/ Disengae the motors. enable = True/ False
    def engage_motor(self, motor_id, enable):
        for dxl_id in motor_id:

            # fault handelling
            while(True):
                dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_ENABLE, enable)
                if(self.okay(motor_id)):
                    break
                else:
                    print('dxl%d: Error with ADDR_MX_TORQUE_ENABLE. Retrying ...' %dxl_id, flush=True)
                    time.sleep(0.25)


    # Returns pos in radians and velocity in radian/ sec
    def get_pos_vel_old(self, motor_id):

        dxl_present_position = []
        dxl_present_velocity = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)

        # Retrieve data
        for i in range(self.n_motors):
            dxl_id = motor_id[i]
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POS_VEL, LEN_MX_PRESENT_POS_VEL)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead get_pos_vel failed" % (dxl_id))
                dxl_present_position.append(float('nan'))
                dxl_present_velocity.append(float('nan'))
                # quit()
            else:
                dxl_present_position.append(dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION))
                dxl_present_velocity.append(dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY))

        dxl_present_velocity = np.array(dxl_present_velocity)
        for i in range(len(dxl_present_velocity)):
            if(dxl_present_velocity[i]>=1024):
                dxl_present_velocity[i] = -1.*(dxl_present_velocity[i] - 1024)
        return POS_SCALE*np.array(dxl_present_position), VEL_SCALE*np.array(dxl_present_velocity)

    
    # Returns pos in radians and velocity in radian/ sec
    def get_pos_vel(self, motor_id):

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)
        if(not self.okay(motor_id)):
            print("try one more time. If not, we will spoof packets below ====================== ")
            # try one more time. If not, we will spoof packets below.
            dynamixel.groupBulkReadTxRxPacket(self.group_pos_vel)

        dxl_errored = []
        # Retrieve data
        for i in range(self.n_motors):
            dxl_id = motor_id[i]
            
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POS_VEL, LEN_MX_PRESENT_POS_VEL)).value
            if dxl_getdata_result != 1:
                #send last known values
                dxl_errored.append(dxl_id)

                self.dxl_present_position[i] = self.dxl_last_position[i].copy()
                self.dxl_present_velocity[i] = self.dxl_last_velocity[i].copy()
            else:
                self.dxl_last_position[i] = self.dxl_present_position[i].copy()
                self.dxl_last_velocity[i] = self.dxl_present_velocity[i].copy()

                dxl_present_position = dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)
                dxl_present_velocity = dynamixel.groupBulkReadGetData(self.group_pos_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)
                if(dxl_present_velocity>=1024):
                    dxl_present_velocity = -1.*(dxl_present_velocity - 1024)

                self.dxl_present_position[i] = POS_SCALE*dxl_present_position
                self.dxl_present_velocity[i] = VEL_SCALE*dxl_present_velocity
        
        if len(dxl_errored):
            self.okay(motor_id)
            print("groupBulkRead get_pos_vel failed. Sending last known values for dynamixel ids: " + str(dxl_errored),flush=True)
        return self.dxl_present_position.copy(), self.dxl_present_velocity.copy()


    # Returns pos in radians
    def get_pos(self, motor_id):
        dxl_present_position = []

        # Bulkread present positions
        dynamixel.groupBulkReadTxRxPacket(self.group_pos)

        # Retrieve data
        for dxl_id in motor_id:
            # Get present position value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_pos, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead pos_getdata failed" % (dxl_id))
                dxl_present_position.append(0)
                quit()
            else:
                dxl_present_position.append(dynamixel.groupBulkReadGetData(self.group_pos, dxl_id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION))

        return POS_SCALE*np.array(dxl_present_position)


    # Returns vel in radians/sec
    def get_vel(self, motor_id):
        dxl_present_velocity = []

        dynamixel.groupBulkReadTxRxPacket(self.group_vel)
        for dxl_id in motor_id:
            # Get present velocity value
            dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupBulkReadIsAvailable(self.group_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY)).value
            if dxl_getdata_result != 1:
                print("[ID:%03d] groupBulkRead vel_getdata failed" % (dxl_id))
                dxl_present_velocity.append(0)
                quit()
            else:
                dxl_present_velocity.append(dynamixel.groupBulkReadGetData(self.group_vel, dxl_id, ADDR_MX_PRESENT_VELOCITY, LEN_MX_PRESENT_VELOCITY))

        dxl_present_velocity = np.array(dxl_present_velocity)
        for i in range(len(dxl_present_velocity)):
            if(dxl_present_velocity[i]>=1024):
                dxl_present_velocity[i] = -1.*(dxl_present_velocity[i] - 1024)

        return VEL_SCALE*dxl_present_velocity


    # Returns pos in radians
    def getIndividual_pos(self, motor_id):
        dxl_present_position = []
        
        # Read present position and velocity
        for dxl_id in motor_id:
            dxl_present_position.append(dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_POSITION))
        if (not self.okay(motor_id)):
            self.close(motor_id)
            quit('error getting ADDR_MX_PRESENT_POSITION')
        return POS_SCALE*np.array(dxl_present_position)


    # Returns vel in radians/sec
    def getIndividual_vel(self, motor_id):
        dxl_present_velocity = []
        # Read present position
        for dxl_id in motor_id:
            dxl_present_velocity.append(dynamixel.read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_PRESENT_VELOCITY))
        if (not self.okay(motor_id)):
            self.close(motor_id)
            quit('error getting ADDR_MX_PRESENT_VELOCITY')

        dxl_present_velocity = np.array(dxl_present_velocity)

        for i in range(len(dxl_present_velocity)):
            if(dxl_present_velocity[i]>=1024):
                dxl_present_velocity[i] = -1.*(dxl_present_velocity[i] - 1024)

        return VEL_SCALE*dxl_present_velocity


    # Expects des_pos in radians
    def setIndividual_des_pos(self, motor_id, des_pos_inRadians):
        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay(motor_id)):
                self.close(motor_id)
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

        # Write goal position
        for i in range(len(motor_id)):
            dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION, motor_id[i], ADDR_MX_GOAL_POSITION, int(des_pos_inRadians[i]/POS_SCALE))
        if (not self.okay(motor_id)):
            self.close(motor_id)
            quit('error setting ADDR_MX_GOAL_POSITION =====')


    # Expects des_pos in radians (0-2*pi)
    def set_des_pos(self, motor_id, des_pos_inRadians):

        des_pos_inRadians = np.clip(des_pos_inRadians, 0.0, 2*np.pi)
        # if in torque mode, activate position control mode
        if(self.ctrl_mode == TORQUE_ENABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_DISABLE)
            if (not self.okay(motor_id)):
                self.close(motor_id)
                quit('error disabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_DISABLE

        # Write goal position
        for i in range(len(motor_id)):
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.group_desPos, motor_id[i], int(des_pos_inRadians[i]/POS_SCALE), LEN_MX_GOAL_POSITION)).value
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (motor_id[i]))
                self.close(motor_id)
                quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(self.group_desPos)
        if(not self.okay(motor_id)):
                self.close(motor_id)
                quit('error bulk commanding desired positions')

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(self.group_desPos)


    # Set desired torques, only for MX64
    def set_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        if(self.ctrl_mode == TORQUE_DISABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_ENABLE)
            if (not self.okay(motor_id)):
                self.close(motor_id)
                quit('error enabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_ENABLE

        # Write goal position
        for i in range(len(motor_id)):
            dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.group_desTor, motor_id[i], int(des_tor[i]), LEN_MX_GOAL_TORQUE)).value
            if dxl_addparam_result != 1:
                print(dxl_addparam_result)
                print("[ID:%03d] groupSyncWrite addparam failed" % (motor_id[i]))
                self.close(motor_id)
                quit()

        # Syncwrite goal position
        dynamixel.groupSyncWriteTxPacket(self.group_desTor)
        if(not self.okay(motor_id)):
            self.close(motor_id)
            quit('error bulk commanding desired torques')

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(self.group_desTor)


    # Set desired torques, only for MX64
    def setIndividual_des_torque(self, motor_id, des_tor):
        # If in position mode, activate torque mode
        if(self.ctrl_mode == TORQUE_DISABLE):
            for dxl_id in motor_id:
                dynamixel.write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_TORQUE_CONTROL_MODE, TORQUE_ENABLE)
            if (not self.okay(motor_id)):
                self.close(motor_id)
                quit('error enabling ADDR_MX_TORQUE_CONTROL_MODE')
            self.ctrl_mode = TORQUE_ENABLE

        # Write goal position
        for i in range(len(motor_id)):
            dynamixel.write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, motor_id[i], ADDR_MX_GOAL_TORQUE, int(des_tor[i]))
        if (not self.okay(motor_id)):
            self.close(motor_id)
            quit('error setting ADDR_MX_GOAL_TORQUE')


    # Set maximum velocity
    def set_max_vel(self, motor_id, max_vel):
        for dxl_id in motor_id:
            dynamixel.write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION, dxl_id, ADDR_MX_MAX_VELOCITY, max_vel)
            if (not self.okay(motor_id)):
                self.close(motor_id)
                quit('error setting ADDR_MX_MAX_VELOCITY')

    # Close connection
    def close(self, motor_id):
        # Disengage Dynamixels
        self.engage_motor(motor_id, False)

        # Close port
        dynamixel.closePort(self.port_num)

        return True



if __name__ == '__main__':
    
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE*POS_SCALE, DXL_MAXIMUM_POSITION_VALUE*POS_SCALE]         # Goal position
    index = 0
    
    dxl_ids =  [1, 2]
    # dxl_ids =  [10, 11, 12, 20, 21, 22, 30, 31, 32, 50]
    dy = dxl(dxl_ids)

    # Test timing ==================================
    cnt = 10
    print("Testing timings (avg over %d trials for %d motors) -------------"% (cnt, len(dxl_ids)))
    t_s = time.time()
    for i in range(cnt):
        dxl_present_position, dxl_present_velocity = dy.get_pos_vel(dxl_ids)
    t_e = time.time()
    print("Bulk [pos+vel] read takes:\t\t %0.5f sec **BEST**" % ((t_e-t_s)/float(cnt)))

    t_s = time.time()
    for i in range(cnt):
        dxl_present_position = dy.get_pos(dxl_ids)
        dxl_present_velocity = dy.get_vel(dxl_ids)
    t_e = time.time()
    print("Bulk [pos,vel] read takes:\t\t %0.4f sec" % ((t_e-t_s)/float(cnt)))
    
    t_s = time.time()
    for i in range(cnt):
        dxl_present_position = dy.getIndividual_pos(dxl_ids)
        dxl_present_velocity = dy.getIndividual_vel(dxl_ids)
    t_e = time.time()
    print("Individual [pos,vel] read takes:\t %0.5f sec" % ((t_e-t_s)/float(cnt)))
    
    t_s = time.time()
    for i in range(cnt):
        dxl_present_position, dxl_present_velocity = dy.get_pos_vel(dxl_ids)
        dy.set_des_pos(dxl_ids, dxl_present_position)
    t_e = time.time()
    print("Bulk [pos+vel,ctrl] loop takes:\t\t %0.5f sec **BEST**" % ((t_e-t_s)/float(cnt)))

    t_s = time.time()
    for i in range(cnt):
        dxl_present_position = dy.get_pos(dxl_ids)
        dxl_present_velocity = dy.get_vel(dxl_ids)
        dy.set_des_pos(dxl_ids, dxl_present_position)
    t_e = time.time()
    print("Bulk [pos,vel,ctrl] loop takes:\t\t %0.5f sec" % ((t_e-t_s)/float(cnt)))

    t_s = time.time()
    for i in range(cnt):
        dxl_present_position = dy.getIndividual_pos(dxl_ids)
        dxl_present_velocity = dy.getIndividual_vel(dxl_ids)
        dy.setIndividual_des_pos(dxl_ids, dxl_present_position)
    t_e = time.time()
    print("Individual [pos,vel,ctrl] takes:\t %0.5f sec" % ((t_e-t_s)/float(cnt)))


    # Test reads ==================================
    dy.engage_motor(dxl_ids, False)
    print("Motor disengaged")
    print("Testing position and velocity --------------")
    input("Press enter and apply external forces")
    for i in range(cnt):
        dxl_present_position, dxl_present_velocity = dy.get_pos_vel(dxl_ids)
        for j in range(len(dxl_ids)):
            print("cnt:%03d, dxl_id:%01d ==> Pos:%2.2f, Vel:%1.3f" % (i, dxl_ids[j], dxl_present_position[j], dxl_present_velocity[j]))
    dy.engage_motor(dxl_ids, True)
    # dy.set_max_vel(dxl_ids, 0) # 1-1023, 0:max_rpm
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
                print("cnt:%03d, dxl_id:%01d ==> Pos:%2.2f, Vel:%1.3f" % (i, dxl_ids[j], dxl_present_position[j], dxl_present_velocity[j]))
        
        print("Zero Torque")
        dy.set_des_torque(dxl_ids, 0)
        time.sleep(1)

        print("Max CW Torque")
        dy.set_des_torque(dxl_ids, DXL_MAX_CW_TORQUE_VALUE)
        for i in range(50):
            dxl_present_position = dy.get_pos(dxl_ids)
            for j in range(len(dxl_ids)):
                print("cnt:%03d, dxl_id:%01d ==> Pos:%2.2f, Vel:%1.3f" % (i, dxl_ids[j], dxl_present_position[j], dxl_present_velocity[j]))
      
        
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
        dy.set_des_pos(dxl_ids, dxl_goal_position[index]*np.ones(len(dxl_ids)))


        # wait and read sensors
        while 1:
            dxl_present_position = dy.get_pos(dxl_ids)
            dxl_present_velocity = dy.get_vel(dxl_ids)
            
            for j in range(len(dxl_ids)):
                print("cnt:%03d, dxl_id:%01d ==> GoalPos:%2.2f, Pos:%2.2f, Vel:%1.3f" % (cnt, dxl_ids[j], dxl_goal_position[index], dxl_present_position[j], dxl_present_velocity[j]))

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
