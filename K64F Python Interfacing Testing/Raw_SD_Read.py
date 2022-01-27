import numpy as np

f = open("D:\\numbers.txt", "r")

# file is the open file handle, data_type is the np.TYPE we want to read e.g. uint16 or float 
def Read_SD_Raw_Data(file, data_type):
    a = np.fromfile(file, dtype=data_type)
    data = []
    if (len(a) % 8 == 0):
        Num_readings = (int)(len(a)/8)
    else:
        raise Exception ("Something wrong with data")
    for x in range(Num_readings):
        row = list(a[x*8:x*8+8])
        data.append(row)
    for x in data:
        print(x)

""" a = np.fromfile(f, dtype=np.float32)
data = []

if len(a) % 8 == 0:
    Num_readings = (int)(len(a)/8)
else:
    raise Exception ("Something wrong with data")

for x in range(Num_readings):
    row = list(a[x*8:x*8+8])
    data.append(row)
for x in data:
    print(x) """

""" a = np.fromfile(f, dtype=np.uint16)
# print(a)
print(len(a))
data = []

row1 = a[0:8]  # 8 Not Included !!!!
print(row1)

row2 = a[8:16]
print(row2)

for x in range(10):
    row = list(a[x*8:x*8+8])
    data.append(row)
for x in data:
    print(x) """
