from ctypes.wintypes import HANDLE
from serial.serialutil import SerialException
import serial.tools.list_ports as port_list
import serial
import time


#### How will we convey COM_PORT - very important for this serial connection
#### Don't forget to check if 115200 has any effect on performance or if overhead dominates for such short transfers  


class Serial_K64F:
    Num_Mbed_Devices = 0
    COM_PORTS = []
    connectionType = []  # Create a unique value for USB K64F devices which trigger new functions  
    # Say 11 = mbed USB, 10 = mbed ANY, 12 = mbed TCP, 14 = mbed WIFI 
    VID_PID = []   # USB VID:PID are the Vendor/Product ID respectively - smae for each K64F Board? - You can determine HIC ID from last 8 digits of Serial Number? 
    # Note that the 0240 at the start of Serial Number refers to the K64F Family 
    ID_USB = []   # ID_USB will be the USB serial number - should be unique
    Baud_Rate = []  # For now assume all operating at 9600 - may change later so might need to add later on 
    # IP = []  # Don't think we need this for USB Serial(Mbed) devices 

    # Assume each instance of this class has no more than 1 conected devices 
    Connected_Device_Index = None # Store the index of the connected device, to gather info from above lists 
    Serial_Device = None # Open Serial Port of the Connected Device
    IsConnected = False  

    def List_All_Mbed_USB_Devices(self):
        ports = list(port_list.comports())
        Num_Serial_Devices = len(ports)
        if Num_Serial_Devices > 0:
            for i in range(Num_Serial_Devices):
                COM_Port = ports[i].usb_description()   # ports[i].device outputs COM_PORT    (Note port[i][0][0:16]  is a particular device - port[i][0] is the COM Port of the device)
                if(ports[i][1].startswith("mbed Serial Port")):     # port[i] is a particular device - port[i][1] is the description of the device - port[i][1][0:16] are the characters containing the mbed Serial Port description
                    default_baudrate = 9600 # Assume all boards use default baudrate of 9600 
                    try:
                        Temp_Serial_device = serial.Serial(port=COM_Port, baudrate=default_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
                    except:
                        raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                    # How can we/Do we need to check we have actually connected to device - and that it is meant to be used for what we are using it for 
                    if(not Temp_Serial_device.readable()):
                        raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                    self.Num_Mbed_Devices += 1
                    self.COM_PORTS.append(COM_Port)
                    USB_INFO = ports[i].usb_info().split('=') # USB-PID should be Unique 
                    USB_VIDPID = USB_INFO[1].split(' ')[0]
                    self.VID_PID.append(USB_VIDPID)
                    USB_Serial_Number = USB_INFO[2].split(' ')[0]
                    self.ID_USB.append(USB_Serial_Number)
                    self.connectionType.append(11)     # Added 10 onto definitions used by LJM library to avoid mixing up - however can change if confusing 
                    Temp_Serial_device.close()    # Close COM Port communication once info obtained
               
                elif(ports[i][1].startswith("USB Serial Device")): # For laptop without specific mbed drivers
                    default_baudrate = 9600 # Assume all boards use default baudrate of 9600 
                    try:
                        Temp_Serial_device = serial.Serial(port=COM_Port, baudrate=default_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
                    except:
                        raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                    # How can we/Do we need to check we have actually connected to device - and that it is meant to be used for what we are using it for 
                    if(not Temp_Serial_device.readable()):
                        raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                    self.Num_Mbed_Devices += 1
                    self.COM_PORTS.append(COM_Port)
                    USB_INFO = ports[i].usb_info().split('=') # USB-PID should be Unique 
                    USB_VIDPID = USB_INFO[1].split(' ')[0]
                    self.VID_PID.append(USB_VIDPID)
                    USB_Serial_Number = USB_INFO[2].split(' ')[0]
                    self.ID_USB.append(USB_Serial_Number)
                    self.connectionType.append(11)     # Added 10 onto definitions used by LJM library to avoid mixing up - however can change if confusing 
                    Temp_Serial_device.close()    # Close COM Port communication once info obtained

    def Connect_To_USB_Device(self, Device_Index, Serial_baudrate = 9600):
        if(self.IsConnected):
            print("Already Connected to a Device")
            return
        try:
            self.Serial_Device = serial.Serial(port=self.COM_PORTS[Device_Index], baudrate=Serial_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
            self.Connected_Device_Index = Device_Index
            self.IsConnected = True
        except ValueError:
            print("Invalid parameters")
        except SerialException:
            print("Could not connect to Device")
        except: 
            raise Exception ("Unknown Error Connecting to Device")

    def Disconnect_Usb_Device(self):
        self.Serial_Device.close()
        self.IsConnected = False
        self.Connected_Device_Index = None
    
    # Instructions + Expected Number of Bytes Returned 
    _Read_32_Reg_Instruction = "30" # 0x30
    _Expected_Bytes_Read_32_Reg = 5 # 4 bytes + '/n'
    _Read_16_Reg_Instruction = "31"  # 0x31
    _Expected_Bytes_Read_16_Reg = 3 # 2 bytes + '/n'
    _Read_8_Reg_Instruction = "32" # 0x32
    _Expected_Bytes_Read_8_Reg = 2 # 1 byte + '/n'

    _Write_32_Reg_Instruction = "33" # 0x33
    _Expected_Bytes_Write_32_Reg = 5 # 4 bytes + '/n'
    _Write_16_Reg_Instruction = "34" # 0x34
    _Expected_Bytes_Write_16_Reg = 3 # 2 bytes + '/n'
    _Write_8_Reg_Instruction = "35" # 0x35
    _Expected_Bytes_Write_8_Reg = 2 # 1 byte + '/n'

    def Hex_To_Dec(input):  # input of form "40048024"
        return int(input, 16)
    
    def Hex_To_Bin(input): # input of form "40048024"
        return bin(int(input, 16))

    def Hex_To_Bytes(input): # input of form "40048024"
        return bytes.fromhex(input)


    # Uses private function - returns data written to Register in hex form 
    def Write_32_Reg(self, Target_Address, Hex_Value):
        if not self.IsConnected:
            raise Exception ("No connected mbed Device")
        if(len(Target_Address) != 10 or not Target_Address.startswith("0x")):
            raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")
        if(len(Hex_Value) > (4*2 + 2) or not Hex_Value.startswith("0x")):
            raise Exception ("Trying to Write an Invalid value to mbed Register")
        Target_Address = Target_Address[2:] # Remove "0x"
        Hex_Value = Hex_Value[2:] # Remove "0x"
        Reg_Contents = self._Serial_Write(self._Write_32_Reg_Instruction, Target_Address, Hex_Value, self._Expected_Bytes_Write_32_Reg)
        if(len(Reg_Contents) != 2*(self._Expected_Bytes_Write_32_Reg - 1)): # Expect 32 bits returned 
            raise Exception ("0x33 Issue with Returned Data")
        if(Reg_Contents != Hex_Value): # We expect to recieve the same value we wrote to an address on the mbed
            raise Exception ("0x33 Issue with Returned Data")
        return Reg_Contents  # Hex Form 

    def Write_16_Reg(self, Target_Address, Hex_Value):
        if not self.IsConnected:
            raise Exception ("No connected mbed Device")
        if(len(Target_Address) != 10 or not Target_Address.startswith("0x")):
            raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")
        if(len(Hex_Value) > (2*2 + 2) or not Hex_Value.startswith("0x")):
            raise Exception ("Trying to Write an Invalid value to mbed Register")
        Target_Address = Target_Address[2:] # Remove "0x"
        Hex_Value = Hex_Value[2:] # Remove "0x"
        Reg_Contents = self._Serial_Write(self._Write_16_Reg_Instruction, Target_Address, Hex_Value, self._Expected_Bytes_Write_16_Reg)
        if(len(Reg_Contents) != 2*(self._Expected_Bytes_Write_16_Reg - 1)): # Expect 32 bits returned (subtract the EOL byte)
            raise Exception ("0x34 Issue with Returned Data")
        if(Reg_Contents != Hex_Value):
            raise Exception ("0x34 Issue with Returned Data")
        return Reg_Contents  # Hex Form 

    def Write_8_Reg(self, Target_Address, Hex_Value):
        if not self.IsConnected:
            raise Exception ("No connected mbed Device")
        if(len(Target_Address) != 10 or not Target_Address.startswith("0x")):
            raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")
        if(len(Hex_Value) > (1*2 + 2) or not Hex_Value.startswith("0x")):
            raise Exception ("Trying to Write an Invalid value to mbed Register")
        Target_Address = Target_Address[2:] # Remove "0x"
        Hex_Value = Hex_Value[2:] # Remove "0x"
        Reg_Contents = self._Serial_Write(self._Write_8_Reg_Instruction, Target_Address, Hex_Value, self._Expected_Bytes_Write_8_Reg)
        if(len(Reg_Contents) != 2*(self._Expected_Bytes_Write_8_Reg - 1)): # Expect 32 bits returned (subtract the EOL byte)
            raise Exception ("0x35 Issue with Returned Data")
        if(Reg_Contents != Hex_Value):
            raise Exception ("0x35 Issue with Returned Data")
        return Reg_Contents  # Hex Form 

    # Uses private function - returns data contained in Register in hex form 
    def Read_32_Reg(self, Target_Address): # Target_Address in form of "0x40048024"
        if not self.IsConnected:
            raise Exception ("No connected mbed Device")
        if(len(Target_Address) != 10 or not Target_Address.startswith("0x")):
            raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")
        Target_Address = Target_Address[2:]
        Reg_Contents = self._Serial_Read(self._Read_32_Reg_Instruction, Target_Address, self._Expected_Bytes_Read_32_Reg)
        if(len(Reg_Contents) != 2*(self._Expected_Bytes_Read_32_Reg - 1)): # Expect 32 bits returned 
            raise Exception ("0x30 Issue with Returned Data")
        return Reg_Contents  # Hex Form 

    def Read_16_Reg(self, Target_Address): # Target_Address in form of "0x40048024"
        if not self.IsConnected:
            raise Exception ("No connected mbed Device")
        if(len(Target_Address) != 10 or not Target_Address.startswith("0x")):
            raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")
        Target_Address = Target_Address[2:]
        Reg_Contents = self._Serial_Read(self._Read_16_Reg_Instruction, Target_Address, self._Expected_Bytes_Read_16_Reg)
        if(len(Reg_Contents) != 2*(self._Expected_Bytes_Read_16_Reg - 1)): # Expect 16 bits returned 
            raise Exception ("0x31 Issue with Returned Data")
        return Reg_Contents  # Hex Form

    def Read_8_Reg(self, Target_Address): # Target_Address in form of "0x40048024"
        if not self.IsConnected:
            raise Exception ("No connected mbed Device")
        if(len(Target_Address) != 10 or not Target_Address.startswith("0x")):
            raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")
        Target_Address = Target_Address[2:]
        Reg_Contents = self._Serial_Read(self._Read_8_Reg_Instruction, Target_Address, self._Expected_Bytes_Read_8_Reg)
        if(len(Reg_Contents) != 2*(self._Expected_Bytes_Read_8_Reg - 1)): # Expect 8 bits returned 
            raise Exception ("0x32 Issue with Returned Data")
        return Reg_Contents  # Hex Form

    def _Serial_Read(self, Instruction, Target_Address, Expected_Bytes):
        # Assume arguments are valid for given instruction - check in public method
        # Assume Target Address & Instructions are in form "40048024", "30" - no "0x" prefix
        # Expected_Bytes inludes the '\n' we expect at end of communication
        
        byte_Command = (b'%b%b\n' % (bytes.fromhex(Instruction), bytes.fromhex(Target_Address)))   
        print(byte_Command, Expected_Bytes)
        print(len(byte_Command))
        return self._Serial_Command_Response(byte_Command, Expected_Bytes)

    def _Serial_Write(self, Instruction, Target_Address, Hex_Value, Expected_Bytes):
        # Assume arguments are valid for given instruction - check in public method
        # Assume Target Address, Hex_value & Instructions are in form "40048024", "30" - no "0x" prefix
        # Expected_Bytes inludes the '\n' we expect at end of communication
        # Writes the Hex_Value to the Targeted_Address 

        byte_Command = (b'%b%b%b\n' % (bytes.fromhex(Instruction), bytes.fromhex(Target_Address), bytes.fromhex(Hex_Value)))
        return self._Serial_Command_Response(byte_Command, Expected_Bytes)

    def _Serial_Command_Response(self, byte_Command, Expected_Bytes):
        try:
            num1 = self.Serial_Device.write(byte_Command)   # num1 is number of bytes sent 
        except:
            raise Exception ("Error writing to mbed Serial Device")
        serialString = "" # Used to hold data coming over UART
        # NEED TO ADD A TIMEOUT TO FOLLOWING LINE


        # Removing while(1) should allow the read(size=Expected_Bytes) to naturally timeout after configured time 

        ''' Don't need this chucnk since we know how many bytes we expect to get back 
            Therefore we can use the builtin timeout from the pyserial library  
        while(1):
            print(self.Serial_Device.in_waiting)
            if self.Serial_Device.in_waiting > 0:
                # Read data out of the buffer until a carriage return / new line is found
                # Note there is an inherant issue with this where, if a transmitted value has the equivalent hex value of 0a, it triggers the end of line
                # If we are sending unsigned bytes across channel, inevitably some will have the value of '\n' character 
                # serialString = serialPort.readline()
        '''

        serialString = self.Serial_Device.read(size=Expected_Bytes)
        serialString = serialString.hex()  # Decode bytes into raw hex values
        # if(serialString[Expected_Bytes-1] != 10):    # 10 = 0x0a = '\n' LF character in Ascii 
        if(serialString[-2: ] != '0a'):    # 10 = 0x0a = '\n' LF character in Ascii 
            raise Exception ("Issue with Received Data") # Need to implement proper error handling
        else:
            return serialString[:-2]

     


# Testing 

mbed_Serial_Object = Serial_K64F()
mbed_Serial_Object.List_All_Mbed_USB_Devices()
print(mbed_Serial_Object.Num_Mbed_Devices)
mbed_Serial_Object.Connect_To_USB_Device(0)
# print(mbed_Serial_Object.IsConnected)
# print(mbed_Serial_Object.COM_PORTS[0])
# print(mbed_Serial_Object.Serial_Device)
print(mbed_Serial_Object.Read_32_Reg("0x40048024"))
print(mbed_Serial_Object.Read_16_Reg("0x40048024"))
print(mbed_Serial_Object.Read_8_Reg("0x40048024"))
'''
Serial_device = serial.Serial(port="COM4", baudrate=9600, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
Target_Register = "0x40048024"
Received_String = Read_K64F_Hex_Register(Serial_device, Target_Register, 4)
print("READ COMMAND (0x30): Requested Register = %s; Contents of Register(Hex) = %s" % (Target_Register , Received_String[:-2]))
'''
