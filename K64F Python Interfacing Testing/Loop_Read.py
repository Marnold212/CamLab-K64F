import numpy as np
from serial.serialutil import SerialException
import serial.tools.list_ports as port_list
import serial
import time

def Hex_To_Dec(input):  # input of form "40048024"
    return int(input, 16)

def Hex_To_Bin(input): # input of form "40048024"
    return bin(int(input, 16))

def Hex_To_Bytes(input): # input of form "40048024"
    return bytes.fromhex(input)

# def List_All_Mbed_USB_Devices(self):
def List_All_Mbed_USB_Devices():
    ports = list(port_list.comports())
    Num_Serial_Devices = len(ports)
    Num_Mbed_Devices = 0
    COM_PORTS = []
    connectionType = []  # Create a unique value for USB K64F devices which trigger new functions  
    # Say 11 = mbed USB, 10 = mbed ANY, 12 = mbed TCP, 14 = mbed WIFI 
    VID_PID = []   # USB VID:PID are the Vendor/Product ID respectively - smae for each K64F Board? - You can determine HIC ID from last 8 digits of Serial Number? 
    # Note that the 0240 at the start of Serial Number refers to the K64F Family 
    ID_USB = []   # ID_USB will be the USB serial number - should be unique
    Baud_Rate = []  # For now assume all operating at 9600 - may change later so might need to add later on 
    # IP = []  # Don't think we need this for USB Serial(Mbed) devices 
    if Num_Serial_Devices > 0:
        for i in range(Num_Serial_Devices):
            COM_Port = ports[i].usb_description()   # ports[i].device outputs COM_PORT    (Note port[i][0][0:16]  is a particular device - port[i][0] is the COM Port of the device)
            if(ports[i][1].startswith("mbed Serial Port")):     # port[i] is a particular device - port[i][1] is the description of the device - port[i][1][0:16] are the characters containing the mbed Serial Port description
                default_baudrate = 115200 # Assume all boards use default baudrate of 9600 
                try:
                    Serial_device = serial.Serial(port=COM_Port, baudrate=default_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
                except:
                    raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                # How can we/Do we need to check we have actually connected to device - and that it is meant to be used for what we are using it for 
                if(not Serial_device.readable()):
                    raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                Num_Mbed_Devices += 1
                COM_PORTS.append(COM_Port)
                USB_INFO = ports[i].usb_info().split('=') # USB-PID should be Unique 
                USB_VIDPID = USB_INFO[1].split(' ')[0]
                VID_PID.append(USB_VIDPID)
                USB_Serial_Number = USB_INFO[2].split(' ')[0]
                ID_USB.append(USB_Serial_Number)
                connectionType.append(11)     # Added 10 onto definitions used by LJM library to avoid mixing up - however can change if confusing 
                Serial_device.close()    # Close COM Port communication once info obtained 
            
            if(ports[i][1].startswith("USB Serial Device")):     
                default_baudrate = 115200 # Assume all boards use default baudrate of 9600 
                try:
                    Serial_device = serial.Serial(port=COM_Port, baudrate=default_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
                except:
                    raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                # How can we/Do we need to check we have actually connected to device - and that it is meant to be used for what we are using it for 
                if(not Serial_device.readable()):
                    raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                Num_Mbed_Devices += 1
                COM_PORTS.append(COM_Port)
                USB_INFO = ports[i].usb_info().split('=') # USB-PID should be Unique 
                USB_VIDPID = USB_INFO[1].split(' ')[0]
                VID_PID.append(USB_VIDPID)
                USB_Serial_Number = USB_INFO[2].split(' ')[0]
                ID_USB.append(USB_Serial_Number)
                connectionType.append(11)     # Added 10 onto definitions used by LJM library to avoid mixing up - however can change if confusing 
                Serial_device.close()    # Close COM Port communication once info obtained 

    return(Num_Mbed_Devices, COM_PORTS, connectionType, ID_USB, VID_PID)


def _Serial_Read_Raw(Serial_Device, Expected_Bytes):
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

    serialString = Serial_Device.read(size=Expected_Bytes)
    serialString = serialString.hex()  # Decode bytes into raw hex values
    # if(serialString[Expected_Bytes-1] != 10):    # 10 = 0x0a = '\n' LF character in Ascii 
    if(serialString[-2: ] != '0a'):    # 10 = 0x0a = '\n' LF character in Ascii 
        # raise Exception ("Issue with Received Data") # Need to implement proper error handling
        print("Error", serialString)
    else:
        return serialString[:-2]

# def Reverse_4byte_hex(input):
#     reverse = ""
#     if(len(input) == 4*2):
#         reverse += input[6:8] + input[4:6] + input[2:4] + input[0:2]
#     return reverse

def Reverse_Hex_Byte_Order(input):
    reverse = ""
    if(len(input) %2 == 0): # Should be twice as many hex characters as bytes therefore even number of hex 
        Num_Bytes = len(input) / 2
        x = (int)(Num_Bytes)
        while(x > 0):
            y = 2 * (x - 1)
            reverse += input[y : y+2]
            x -= 1
    return reverse


def ADC_8x_16_Raw_Read(Serial_Device):
    Expected_Bytes = 8*2 + 4 + 1 # Inlcude EOL Char 
    Raw = _Serial_Read_Raw(Serial_Device, Expected_Bytes)
    data = []
    seconds = Hex_To_Dec(Reverse_Hex_Byte_Order(Raw[0:8]))
    for x in range(2, 8+2, 2): # Bytes
        y = x+1   # Due to byte order of device - 
        data.append(Hex_To_Dec(Raw[(4*y) + 0 : (4*y) + 4]))
        data.append(Hex_To_Dec(Raw[(4*x) + 0 : (4*x) + 4]))
    return seconds, data


# Assumes Data recieved is 
def Convert_ADC_Raw(Raw_Reading, ADC_Resolution, Max_Min_Voltage): 
    Signed_Value = np.int16(Raw_Reading)  
    quant_step = (2 * Max_Min_Voltage) / (2**ADC_Resolution)
    return Signed_Value * quant_step


def Decode_Time_In_Secs(HEX_four_bytes):
    return Hex_To_Dec(Reverse_Hex_Byte_Order(HEX_four_bytes))

def Decode_8x16_Raw(HEX_sixteen_bytes):
    if(len(HEX_sixteen_bytes) != 16*2):
        raise ValueError
    Raw_Readings = []
    for x in range(8):
        Raw_Readings.append(Hex_To_Dec(Reverse_Hex_Byte_Order(HEX_sixteen_bytes[4*x : 4*(x+1)])))
    return Raw_Readings

def Decode_6_Compressed_PWM_Duty(HEX_six_bytes):
    if(len(HEX_six_bytes) != 6*2):
        raise ValueError
    Duty_Cycles = []
    for x in range(6):
        Duty_Cycles.append(Hex_To_Dec(HEX_six_bytes[2*x : 2*x + 2]) / 100.)
    return Duty_Cycles 





def Decode_Raw_Data(Raw_Data):
    Results = []
    for sample in Raw_Data:
        entry = []
        time = Decode_Time_In_Secs(sample[0:(4*2)])
        entry.append(time)
        entry.append(Decode_8x16_Raw(sample[4 * 2 : (4 + (2*8)) * 2]))
        entry.append(Decode_6_Compressed_PWM_Duty(sample[(4 + (2*8)) * 2 : 40 + 6*2]))

        Results.append(entry)
    print(Raw_Data)
    return Results
        # for x in range(2, 8+2, 2): # Bytes
        #     y = x+1   # Due to byte order of device - 

        #     entry.append(Hex_To_Dec(sample[(4*y) + 0 : (4*y) + 4]))
        #     entry.append(Hex_To_Dec(sample[(4*x) + 0 : (4*x) + 4]))
        #     # entry.append(Convert_ADC_Raw(Hex_To_Dec(sample[(4*y) + 0 : (4*y) + 4]), 16, 5))
        #     # entry.append(Convert_ADC_Raw(Hex_To_Dec(sample[(4*x) + 0 : (4*x) + 4]), 16, 5))
        # Results.append(entry)
        

# Testing 
if __name__ == "__main__":
    mbed_USB_info = List_All_Mbed_USB_Devices()

    for i in range(5):
        print(mbed_USB_info[i])

    # serial_port = serial.Serial(port=mbed_USB_info[1][0], baudrate=115200, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
    # for x in range(1000):
    #     raw_data = ADC_8x_16_Raw_Read(serial_port)
    #     # raw_data = serial_port.read(1)
    #     data = []
    #     for x in range(8):
    #         data.append(Convert_ADC_Raw(raw_data[1][x], 16, 5))
    #     # print(raw_data)
    #     print(data, raw_data  [0])

    Bytes_Per_Sample = 32
    Number_Samples = 10
    Serial_Baudrate = 576000  # 962100

    serial_port = serial.Serial(port=mbed_USB_info[1][0], baudrate=Serial_Baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
    data = []
    for x in range(Number_Samples):
        raw = serial_port.read(Bytes_Per_Sample).hex()
        data.append(raw)
    # print(data)
    # print(data)
    Formatted = Decode_Raw_Data(data)
    print(Formatted[0:2])
    # print(Results[0:2])

# 
#  Serial_device = serial.Serial(port="COM4", baudrate=9600, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
# Target_Register = "0x40048024"
# Received_String = Read_K64F_Hex_Register(Serial_device, Target_Register, 4)
# print("READ COMMAND (0x30): Requested Register = %s; Contents of Register(Hex) = %s" % (Target_Register , Received_String[:-2]))