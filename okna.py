import tkinter as tk
from cgitb import reset

import serial
import time
import os



def create_widget(parent, widget_type, **options):
    return widget_type(parent, **options)

def create_button(parent, text, fg):
    return create_widget(parent, tk.Button, text=text, fg=fg, bg='lightblue', bd=3, cursor='hand2', highlightcolor='red', highlightthickness=2, highlightbackground='black', relief=tk.RAISED)

window = create_widget(None, tk.Tk)
window.title("PID LR")
window.geometry("12"
                ""
                "00x400")

P_val = tk.StringVar()
I_val = tk.StringVar()
D_val = tk.StringVar()

p_val = tk.StringVar()
i_val = tk.StringVar()
d_val = tk.StringVar()

P_rool_val = tk.StringVar()
I_rool_val = tk.StringVar()
D_rool_val = tk.StringVar()

p_rool_val = tk.StringVar()
i_rool_val = tk.StringVar()
d_rool_val = tk.StringVar()

P_yaw_val = tk.StringVar()
I_yaw_val = tk.StringVar()
D_yaw_val = tk.StringVar()

p_yaw_val = tk.StringVar()
i_yaw_val = tk.StringVar()
d_yaw_val = tk.StringVar()

FDP_val = tk.StringVar()
FDP_D_gain = tk.StringVar()

filepath = os.path.join("D://PID_APP/olddata.txt")
f = open(filepath, "r")


P_val.set(f.readline())
I_val.set(f.readline())
D_val.set(f.readline())

p_val.set(f.readline())
i_val.set(f.readline())
d_val.set(f.readline())

FDP_val.set(f.readline())
FDP_D_gain.set(f.readline())

P_rool_val.set(f.readline())
I_rool_val.set(f.readline())
D_rool_val.set(f.readline())

p_rool_val.set(f.readline())
i_rool_val.set(f.readline())
d_rool_val.set(f.readline())

P_yaw_val.set(f.readline())
I_yaw_val.set(f.readline())
D_yaw_val.set(f.readline())

p_yaw_val.set(f.readline())
i_yaw_val.set(f.readline())
d_yaw_val.set(f.readline())
f.close()





def submit():
    P = P_val.get()
    I = I_val.get()
    D = D_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("P ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)
    ser.write(bytearray("I ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)
    ser.write(bytearray("D ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    #f.write('\n')
    f.write(I_val.get())
    #f.write('\n')
    f.write(D_val.get())

    f.write(p_val.get())
    #f.write('\n')
    f.write(i_val.get())
    #f.write('\n')
    f.write(d_val.get())

    #f.write('\n')
    f.write(FDP_val.get())
    #f.write('\n')
    f.write(FDP_D_gain.get())
    f.close()

def submit_p():
    P = p_val.get()
    #I = I_val.get()
    #D = D_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("p ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)


    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())

    f.close()

def submit_i():
    I = i_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)


    ser.write(bytearray("i ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_d():
    D = d_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)


    ser.write(bytearray("d ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_P():
    P = P_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("P ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_I():
    I = I_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)


    ser.write(bytearray("I ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_D():
    D = D_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)


    ser.write(bytearray("D ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_p_rool():
    P = p_rool_val.get()
    # I = I_val.get()
    # D = D_val.get()
    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("e ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_i_rool():
    I = i_rool_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("g ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_d_rool():
    D = d_rool_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("h ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_P_rool():
    P = P_rool_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("a ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_I_rool():
    I = I_rool_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("b ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_D_rool():
    D = D_rool_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("c ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_p_yaw():
    P = p_yaw_val.get()
    # I = I_val.get()
    # D = D_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("m ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_i_yaw():
    I = i_yaw_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("n ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_d_yaw():
    D = d_yaw_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("o ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_P_yaw():
    P = P_yaw_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("j ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(P, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_I_yaw():
    I = I_yaw_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("k ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(I, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def submit_D_yaw():
    D = D_yaw_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("l", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()


def submit_FDP_Out():
    FDP = FDP_val.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("F ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(FDP, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_val.get())
    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()
def submit_FDP_D_Gain():
    FDP_D = FDP_D_gain.get()

    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("f ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(FDP_D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))
    time.sleep(1)

    f = open(filepath, "w")
    f.write(P_val.get())
    f.write(I_val.get())
    f.write(D_val.get())

    f.write(p_val.get())
    f.write(i_val.get())
    f.write(d_val.get())

    f.write(FDP_D_gain.get())

    f.write(P_rool_val.get())
    f.write(I_rool_val.get())
    f.write(D_rool_val.get())

    f.write(p_rool_val.get())
    f.write(i_rool_val.get())
    f.write(d_rool_val.get())

    f.write(P_yaw_val.get())
    f.write(I_yaw_val.get())
    f.write(D_yaw_val.get())

    f.write(p_yaw_val.get())
    f.write(i_yaw_val.get())
    f.write(d_yaw_val.get())
    f.close()

def Reset():
    FDP_D = FDP_D_gain.get()
    ser = serial.Serial("COM7", 115200, timeout=1)

    ser.write(bytearray("r ", 'ascii'))
    time.sleep(0.4)
    ser.write(bytearray(FDP_D, 'ascii'))
    time.sleep(0.5)
    ser.write(bytearray('\r', 'ascii'))




P_label = tk.Label(window, text = 'P', font=('calibre',10 , 'bold'))
I_label = tk.Label(window, text = 'I', font=('calibre',10 , 'bold'))
D_label = tk.Label(window, text = 'D', font=('calibre',10 , 'bold'))

p_label = tk.Label(window, text = 'P_AR', font=('calibre',10 , 'bold'))
i_label = tk.Label(window, text = 'I_AR', font=('calibre',10 , 'bold'))
d_label = tk.Label(window, text = 'D_AR', font=('calibre',10 , 'bold'))

P_rool_label = tk.Label(window, text = 'P_rool', font=('calibre',10 , 'bold'))
I_rool_label = tk.Label(window, text = 'I_rool', font=('calibre',10 , 'bold'))
D_rool_label = tk.Label(window, text = 'D_rool', font=('calibre',10 , 'bold'))

p_rool_label = tk.Label(window, text = 'P_AR_rool', font=('calibre',10 , 'bold'))
i_rool_label = tk.Label(window, text = 'I_AR_rool', font=('calibre',10 , 'bold'))
d_rool_label = tk.Label(window, text = 'D_AR_rool', font=('calibre',10 , 'bold'))

P_yaw_label = tk.Label(window, text = 'P_yaw', font=('calibre',10 , 'bold'))
I_yaw_label = tk.Label(window, text = 'I_yaw', font=('calibre',10 , 'bold'))
D_yaw_label = tk.Label(window, text = 'D_yaw', font=('calibre',10 , 'bold'))

p_yaw_label = tk.Label(window, text = 'P_AR_yaw', font=('calibre',10 , 'bold'))
i_yaw_label = tk.Label(window, text = 'I_AR_yaw', font=('calibre',10 , 'bold'))
d_yaw_label = tk.Label(window, text = 'D_AR_yaw', font=('calibre',10 , 'bold'))

FDP_label = tk.Label(window, text = 'FDP_Out', font=('calibre',10 , 'bold'))
FDP_D_label = tk.Label(window, text = 'FDP_D_Gain', font=('calibre',10 , 'bold'))

P_entry = tk.Entry(window,textvariable = P_val, font=('calibre',10,'normal'))
I_entry = tk.Entry(window,textvariable = I_val, font=('calibre',10,'normal'))
D_entry = tk.Entry(window,textvariable = D_val, font=('calibre',10,'normal'))

p_entry = tk.Entry(window,textvariable = p_val, font=('calibre',10,'normal'))
i_entry = tk.Entry(window,textvariable = i_val, font=('calibre',10,'normal'))
d_entry = tk.Entry(window,textvariable = d_val, font=('calibre',10,'normal'))

P_rool_entry = tk.Entry(window,textvariable = P_rool_val, font=('calibre',10,'normal'))
I_rool_entry = tk.Entry(window,textvariable = I_rool_val, font=('calibre',10,'normal'))
D_rool_entry = tk.Entry(window,textvariable = D_rool_val, font=('calibre',10,'normal'))

p_rool_entry = tk.Entry(window,textvariable = p_rool_val, font=('calibre',10,'normal'))
i_rool_entry = tk.Entry(window,textvariable = i_rool_val, font=('calibre',10,'normal'))
d_rool_entry = tk.Entry(window,textvariable = d_rool_val, font=('calibre',10,'normal'))

P_yaw_entry = tk.Entry(window,textvariable = P_yaw_val, font=('calibre',10,'normal'))
I_yaw_entry = tk.Entry(window,textvariable = I_yaw_val, font=('calibre',10,'normal'))
D_yaw_entry = tk.Entry(window,textvariable = D_yaw_val, font=('calibre',10,'normal'))

p_yaw_entry = tk.Entry(window,textvariable = p_yaw_val, font=('calibre',10,'normal'))
i_yaw_entry = tk.Entry(window,textvariable = i_yaw_val, font=('calibre',10,'normal'))
d_yaw_entry = tk.Entry(window,textvariable = d_yaw_val, font=('calibre',10,'normal'))

FDP_entry = tk.Entry(window,textvariable = FDP_val, font=('calibre',10,'normal'))
FDP_D_enter =  tk.Entry(window,textvariable = FDP_D_gain, font=('calibre',10,'normal'))

res_btn=tk.Button(window,text = 'MPU_Reset', command = Reset)

set_btn=tk.Button(window,text = 'Set All', command = submit)
set_P_btn=tk.Button(window,text = 'Set P', command = submit_P)
set_I_btn=tk.Button(window,text = 'Set I', command = submit_I)
set_D_btn=tk.Button(window,text = 'Set D', command = submit_D)

set_p_btn=tk.Button(window,text = 'Set P', command = submit_p)
set_i_btn=tk.Button(window,text = 'Set I', command = submit_i)
set_d_btn=tk.Button(window,text = 'Set D', command = submit_d)

set_P_rool_btn=tk.Button(window,text = 'Set P', command = submit_P_rool)
set_I_rool_btn=tk.Button(window,text = 'Set I', command = submit_I_rool)
set_D_rool_btn=tk.Button(window,text = 'Set D', command = submit_D_rool)

set_p_rool_btn=tk.Button(window,text = 'Set P', command = submit_p_rool)
set_i_rool_btn=tk.Button(window,text = 'Set I', command = submit_i_rool)
set_d_rool_btn=tk.Button(window,text = 'Set D', command = submit_d_rool)

set_P_yaw_btn=tk.Button(window,text = 'Set P', command = submit_P_yaw)
set_I_yaw_btn=tk.Button(window,text = 'Set I', command = submit_I_yaw)
set_D_yaw_btn=tk.Button(window,text = 'Set D', command = submit_D_yaw)

set_p_yaw_btn=tk.Button(window,text = 'Set P', command = submit_p_yaw)
set_i_yaw_btn=tk.Button(window,text = 'Set I', command = submit_i_yaw)
set_d_yaw_btn=tk.Button(window,text = 'Set D', command = submit_d_yaw)


set_FDP_btn=tk.Button(window,text = 'Set FDP_Out', command = submit_FDP_Out)
set_FDP_D_btn = tk.Button(window,text = 'Set FDP_D_Gain', command = submit_FDP_D_Gain)

P_label.grid(row=0,column=0)
I_label.grid(row=1,column=0)
D_label.grid(row=2,column=0)

p_label.grid(row=3,column=0)
i_label.grid(row=4,column=0)
d_label.grid(row=5,column=0)

P_rool_label.grid(row=0,column=4)
I_rool_label.grid(row=1,column=4)
D_rool_label.grid(row=2,column=4)

p_rool_label.grid(row=3,column=4)
i_rool_label.grid(row=4,column=4)
d_rool_label.grid(row=5,column=4)

P_yaw_label.grid(row=0,column=7)
I_yaw_label.grid(row=1,column=7)
D_yaw_label.grid(row=2,column=7)

p_yaw_label.grid(row=3,column=7)
i_yaw_label.grid(row=4,column=7)
d_yaw_label.grid(row=5,column=7)

FDP_label.grid(row=6,column=0)
FDP_D_label.grid(row=7,column=0)

P_entry.grid(row=0,column=2)
I_entry.grid(row=1,column=2)
D_entry.grid(row=2,column=2)

p_entry.grid(row=3,column=2)
i_entry.grid(row=4,column=2)
d_entry.grid(row=5,column=2)

P_rool_entry.grid(row=0,column=5)
I_rool_entry.grid(row=1,column=5)
D_rool_entry.grid(row=2,column=5)

p_rool_entry.grid(row=3,column=5)
i_rool_entry.grid(row=4,column=5)
d_rool_entry.grid(row=5,column=5)


P_yaw_entry.grid(row=0,column=8)
I_yaw_entry.grid(row=1,column=8)
D_yaw_entry.grid(row=2,column=8)

p_yaw_entry.grid(row=3,column=8)
i_yaw_entry.grid(row=4,column=8)
d_yaw_entry.grid(row=5,column=8)


FDP_entry.grid(row=6,column=2)
FDP_D_enter.grid(row=7,column=2)

set_btn.grid(row=10,column=2)

set_P_btn.grid(row=0,column=3)
set_I_btn.grid(row=1,column=3)
set_D_btn.grid(row=2,column=3)

set_p_btn.grid(row=3,column=3)
set_i_btn.grid(row=4,column=3)
set_d_btn.grid(row=5,column=3)

set_P_rool_btn.grid(row=0,column=6)
set_I_rool_btn.grid(row=1,column=6)
set_D_rool_btn.grid(row=2,column=6)

set_p_rool_btn.grid(row=3,column=6)
set_i_rool_btn.grid(row=4,column=6)
set_d_rool_btn.grid(row=5,column=6)

set_P_yaw_btn.grid(row=0,column=9)
set_I_yaw_btn.grid(row=1,column=9)
set_D_yaw_btn.grid(row=2,column=9)

set_p_yaw_btn.grid(row=3,column=9)
set_i_yaw_btn.grid(row=4,column=9)
set_d_yaw_btn.grid(row=5,column=9)

set_FDP_btn.grid(row=6,column=3)
set_FDP_D_btn.grid(row=7,column=3)

res_btn.grid(row = 0, column = 10)



while(1):
    window.update()

