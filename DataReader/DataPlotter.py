
import matplotlib.pyplot as plt
from datetime import datetime

NAME = "main/Log.txt"

file = open(NAME,'r')



temp = []
press = [] 
tim = [] 

while True:
    next_line = file.readline()
    
        
    if not next_line:
        break;

    words = next_line.split(" ")
    print(words[2])
    try:

        temp.append(float(words[2][0:-1]))
        press.append(float(words[3]))
        tim.append(words[1])
            #print(words[5])

    except IndexError:
        pass


    




file.close()



plt.plot(temp)
plt.show()

plt.plot(press)
plt.show()
