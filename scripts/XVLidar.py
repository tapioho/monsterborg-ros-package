import time, sys, traceback, serial
import numpy as np

class XVLidar:
    """ A class for using thes XV LiDAR controller from www.getsurreal.com in a more object oriented way.
        The source code for the controller can be found at: https://github.com/getSurreal/XV_Lidar_Controller
        All the arithmetics for the data package interpreting are taken from the 'XV Lidar Controller Visual Test'-example,
        which can be found here: https://www.getsurreal.com/xv-lidar-controller-first-release/xv-lidar-controller-visual-test/ """
    
    # Input arguments for creating a XVLidar object:
    #   port: name of the port the lidar controller is connected to
    #   baud (optional): symbol rate (baud) for the communication, 115200 by default
    
    def __init__(self, port, baud=115200):
        """Initialize the serial port and also some variables.
            Finally, reset the LiDAR (Restores the original configuration)"""     
        # Serial port
        self.__ser = serial.Serial(port, baud, timeout=1)
        # Data related to the LiDAR
        self.__speedRPM = 0  # Rotational speed of the LiDAR (RPM)
        self.__lidarData = np.zeros((360,2))  # 360x2 numpy array for storing the data
        self.__nErrors = 0  # Number of errors
        
        # Reset the LiDAR
        self.reset()
        print('----------------------------------------------------')
        print("Now connected to device in port: " + port)
        print('----------------------------------------------------')

    def __compute_speed(self, data):
        """Compute the speed from the given data packet."""
        
        # How the arithmetics work:
        # The second symbol of the data package is shifted 8 bits left,
        # both symbols are compared with binary OR,
        # the result is then converted to float and divided by 64.
        speed_rpm = float( data[0] | (data[1] << 8) ) / 64.0
        return speed_rpm

    def __checksum(self, data):
        """Compute and return the checksum as an int.
        data -- list of 20 bytes (as ints), in the order they arrived in.
        """
        # group the data by word, little-endian
        data_list = []
        for t in range(10):
            data_list.append( data[2*t] + (data[2*t+1]<<8) )
        
        # compute the checksum on 32 bits
        chk32 = 0
        for d in data_list:
            chk32 = (chk32 << 1) + d
    
        # return a value wrapped around on 15bits, and truncated to still fit into 15 bits
        checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ) # wrap around to fit into 15 bits
        checksum = checksum & 0x7FFF # truncate to 15 bits
        return int(checksum)

    def __dataFromBytes(self, angle, data):
        """Interpres the data bytes into a more readable form"""
        # Individual bytes
        x0 = data[0]
        x1 = data[1]
        x2 = data[2]
        x3 = data[3]

        dist_mm = x0 | (( x1 & 0x3f) << 8) 
        quality = x2 | (x3 << 8) 
        self.__lidarData[int(angle), 0] = dist_mm
        self.__lidarData[int(angle), 1] = quality
        
        
    def scan(self, duration=1):
        """Perform a scan for given duration (1 second by default)"""

        interpLevel = 0  # A counter to indicate at which level in interpreting we are
        positionIdx = 0  # An index for calculating the angle of the measurement
        startTime = time.time()  # Starting time to track how long the scan has been going on 
        
        while time.time() - startTime < duration:
            try:
                time.sleep(0.00001)
                if interpLevel == 0:
                    # Read a single symbol
                    b = ord(self.__ser.read(1))
                    # Check if the symbol is a start byte
                    if b == 0xFA:
                        interpLevel = 1
                    else:
                        interpLevel = 0
                elif interpLevel == 1:
                    # Position index
                    b = ord(self.__ser.read(1))
                    # Check if the byte is between 160 and 249
                    if b >= 0xA0 and b <= 0xF9:
                        positionIdx = b - 0xA0
                        interpLevel = 2
                    # In case the byte is not between 160 and 249,
                    # and it's not a start byte,
                    # initialize the init_level
                    elif b != 0xFA:
                        interpLevel = 0
                elif interpLevel == 2:
                    # Speed byte
                    b_speed = [ord(b) for b in self.__ser.read(2)]
                    # Data bytes
                    b_data0 = [ord(b) for b in self.__ser.read(4)]
                    b_data1 = [ord(b) for b in self.__ser.read(4)]
                    b_data2 = [ord(b) for b in self.__ser.read(4)]
                    b_data3 = [ord(b) for b in self.__ser.read(4)]
            
                    # The whole data packet
                    all_data = [ 0xFA, positionIdx+0xA0 ] + b_speed + b_data0 + b_data1 + b_data2 + b_data3
                    
                    # Checksum
                    b_checksum = [ ord(b) for b in self.__ser.read(2) ]
                    incoming_checksum = int(b_checksum[0]) + (int(b_checksum[1]) << 8)
            
                    # Verify that the received checksum is equal to the one computed from the data
                    if self.__checksum(all_data) == incoming_checksum:
                        self.__speedRPM = self.__compute_speed(b_speed)
                        self.__dataFromBytes(positionIdx * 4 + 0, b_data0)
                        self.__dataFromBytes(positionIdx * 4 + 1, b_data1)
                        self.__dataFromBytes(positionIdx * 4 + 2, b_data2)
                        self.__dataFromBytes(positionIdx * 4 + 3, b_data3)
                    else:
                        self.__nErrors += 1
        
                    # Initialize the level counter and position index, wait for the next package
                    interpLevel = 0
                    positionIdx = 0
                    
            except:
                traceback.print_exc(file=sys.stdout)
                break
            
    def getPort(self):
        """Returns the ports name the lidar is connected to"""
        return self.__ser.name
    
    def getErrors(self):
        """Returns the number of errors so far"""
        return self.__nErrors
    
    def getSpeed(self):
        """Reads and returns the current rotational speed (RPM)"""
        self.scan()
        return self.__speedRPM

    def getData(self):
        """Returns the current LiDAR data array"""
        return self.__lidarData

    def motorOff(self):
        """Sends the command 'MotorOff' to the serial port,
            which stops the spinning of the lidar """
        # Prefixing the string with 'b' converts it to bytes.
        # This is required for Python 3.x, optional for Python 2.x
        self.__ser.write(b'MotorOff\n')

    def motorOn(self):
        """Sends the command 'MotorOn' to the serial port,
            which starts the spinning of the lidar """
        # Prefixing the string with 'b' converts it to bytes.
        # This is required for Python 3.x, optional for Python 2.x
        self.__ser.write(b'MotorOn\n')

    def setRPM(self, rpm):
        """Sets the desired rotation speed (RPM)
            Min: 180, Max:349"""
        if rpm >= 180 and rpm <= 349:
            self.sendCommand("SetRPM "+str(rpm))
        else:
            print("Unable to set the rotation speed to " + str(rpm))
            print("The rotation speed needs to be between 180 and 349 (RPM)")
    
    def reset(self):
        """Resets the LiDAR and restores the original configuration
            by sending the command 'ResetConfig' to the serial port."""
        # Prefixing the string with 'b' converts it to bytes.
        # This is required for Python 3.x, optional for Python 2.x
        self.__ser.write(b'ResetConfig\n')

    def sendCommand(self, cmd):
        """Sends a command through the serial port into the LiDAR"""
        # The newline character is required to signify the end of a command
        cmd += '\n'
        # Check if Python is 3.x, in which case convert to bytes (Not required for 2.x)
        if sys.version_info[0] == 3:
            cmd = bytes(cmd, 'utf-8')
        self.__ser.write(cmd)


