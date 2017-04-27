import math
import sys

tableSize = int(sys.argv[1])
amplitude = int(sys.argv[2])
offset = amplitude

phases = [2 * math.pi * i / tableSize for i in range(tableSize)]
values = [(offset + amplitude * math.sin(phase)) / 2 for phase in phases]
rounded = [str(int(0.5 + value)) for value in values]

print(', '.join(rounded))
