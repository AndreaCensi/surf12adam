
from threading import Thread
from PIL import Image, ImageTk
import Tkinter as TK
from Tkinter import Label


class UI(Thread):
    self.started = False
    def __init__(self):
        threading.Thread.__init__(self)
        self.ui = TK.Tk()
        
    def run(self):
        self.started = True
        self.ui.mainloop()
    
    def set_images(self, img, imc):
        tk_goal = ImageTk.PhotoImage(pil_goal)
        self.panel_goal = Label(self.ui, border=0, image=tk_goal)
        self.panel_goal.pack()
        
        tk_current = ImageTk.PhotoImage(pil_current)
        self.panel_current = Label(self.ui, border=0, image=tk_current)
        self.panel_current.pack()
