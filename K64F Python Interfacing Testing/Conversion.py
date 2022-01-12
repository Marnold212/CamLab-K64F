import numpy as np 

# If the ADC value returned is in TWO's complimenet, this is the conversion to voltage

def Twos_To_Voltage(twos_value, max_voltage, resolution): # Assuming voltage range is bipolar, so the minimum possible voltage is the negative of the maximum possible voltage
    q_step = (max_voltage * 2) / (2**resolution)  # Assuming range of possible voltages is -V to +V
    return (np.int16(twos_value) * q_step)

Twos_Compliment_Values = []
Signed_Values = []

Resolution = 16
Max_Value = 5
Min_Value = -5

len_list = 1000

for x in range(len_list):
    Twos_Compliment_Values.append(np.random.randint(0, 2**Resolution -1))

for x in range(len_list):
    print(Twos_Compliment_Values[x], Twos_To_Voltage(Twos_Compliment_Values[x], 5, 16))