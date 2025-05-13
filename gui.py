# gui.py
import tkinter as tk
import threading
from main import run_simulation

def create_gui():
    root = tk.Tk()
    root.title("SkyOrchestrator Steuerung")

    start_button = tk.Button(root, text="Simulation starten", command=lambda: threading.Thread(target=run_simulation).start())
    start_button.pack(padx=20, pady=20)

    root.mainloop()

if __name__ == "__main__":
    create_gui()
