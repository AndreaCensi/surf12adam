#import Tkinter
from Tkinter import *
from PIL import Image, ImageTk
#import os, sys
import time

im = Image.open('/home/adam/bild.jpg')
im = im.resize((256,265))
print im

window = Tk()

tkim = ImageTk.PhotoImage(im)
print tkim

w = Label(window, image=tkim)
print w

w.pack()
time.sleep(5)
im2 = Image.open('/home/adam/bild2.jpg')
im2 = im2.resize((256,265))
tkim2 = ImageTk.PhotoImage(im2)

w.configure(image=tkim2)
window.mainloop()
