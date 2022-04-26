#Program for plotting histogram from timestamp mode using TDC1 Time to digital converter

from ctypes import c_int, c_double, c_char_p, create_string_buffer, cdll
from numpy.ctypeslib import ndpointer
import numpy as np
import matplotlib.pyplot as plt
import serial.tools.list_ports


def main():
    
    t_collect = 1.0 #Collect data for 1 second

    #Loads the C library and functions
    lib = cdll.LoadLibrary("/home/pi/lab/timestamp_measurement/calc.so")
    stamp = lib.stamp
    init_device = lib.init_device
    close_device = lib.close_device
    stamp.restype = ndpointer(dtype=c_int, shape=(23,))
    
    port_name = ''
    ports = list(serial.tools.list_ports.comports()) #Checks ports and gets the correct one
    for p in ports:
        if "USB Counter" in p.description:
            port_name = p[0]
            
    if port_name == '':
        raise Exception("Could not find USB counter") #Throws error if USB counter could not be found

    init_device(c_char_p(port_name.encode('utf-8'))) #Initialises device

    fig,ax = plt.subplots()
    plt.ion() #Turns on interactive mode. Allows changes to be updated real time
    
    time_lst = np.arange(-20,22,2,dtype=int) #Creates x values
    histo = np.zeros(21)
    plot1, = ax.plot(time_lst,histo,linestyle='--',marker = '*',color = 'r')

    plt.xticks(np.arange(-20,22,4,dtype=int))
    plt.xlabel("Time delay(ns)",fontsize = 18)
    plt.ylabel("#of Counts",fontsize = 18)
    
    plt.ylim((0,1000))

    time_pos = 0
    time_str = ""
    
    time_pos2 = 0
    time_str2 = ""

    props = dict(boxstyle='square', facecolor='lightcyan', alpha=1)
    box1 = ax.text(-0.15, 1.1,"")
    box2 = ax.text(0.25, 1.1,"")
    box3 = ax.text(0.55, 1.1, "")
    
    old_max = 1000 #y axis maximum value
    while True:
        histo_buf = create_string_buffer(b"",23*4)
        histo = stamp(c_double(t_collect),histo_buf) #Calls the C function

        cnt2 = histo[21]
        cnt4 = histo[22]
        histo = histo[:-2]

        box1.remove() 
        box2.remove()
        box3.remove()
        
        pos_max = np.argsort(histo) #Array that has the indices of histo corresponding to the size of elements in histo
        
        time_pos = pos_max[-1] #Last element of pos_max contains the index for the largest element of histo
        time_str = convert_to_time_str(time_pos)
        
        time_pos2 = pos_max[-2]
        time_str2 = convert_to_time_str(time_pos2)
        
        if histo[time_pos] > old_max or histo[time_pos] < old_max - 400:
            old_max = histo[time_pos] + 200
            ax.set_ylim((0,old_max))

        str1 = "Events channel 2=" + str(cnt2)
        str2 = "Events channel 4=" + str(cnt4)
        str3 = "Counts at " + time_str + " and " + time_str2 + "ns= " + str(histo[time_pos]+histo[time_pos2]) #Good if peak is spread over more than one value
        #str3 = "Maximum counts at " + time_str +  "ns= " + str(histo[time_pos]) #If only one time is interesting

        
        box1 = ax.text(-0.15, 1.1, str1, transform=ax.transAxes, fontsize=18, verticalalignment='top', bbox=props) 
        box2 = ax.text(0.25, 1.1, str2, transform=ax.transAxes, fontsize=18, verticalalignment='top', bbox=props)
        box3 = ax.text(0.7, 1.1, str3, transform=ax.transAxes, fontsize=18, verticalalignment='top', bbox=props)

        plot1.set_ydata(histo)

        try: #This will crash if the figure window is closed
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.001)
        except:
            break

    plt.show()
    close_device() #Closes the device

def convert_to_time_str(pos):
    '''Converts index position of histogram to corresponding time as a string'''
    if pos >= 10:
        time_str = str(2*(pos-10))
    else:
        time_str = "-" + str(2*(10-pos))
        
    return time_str

if __name__ == '__main__':
    main()
