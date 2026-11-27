import tkinter as tk

# 1. Create the main window
root = tk.Tk()
root.title("Tkinter Line Example (macOS)")
root.geometry("400x300")

# 2. Create a Canvas widget to draw on
# Setting a background color helps distinguish the canvas on macOS light/dark mode
canvas = tk.Canvas(root, width=400, height=300, bg="white")
canvas.pack(fill="both", expand=True)

# 3. Draw lines using create_line(x1, y1, x2, y2, options...)
# Solid blue diagonal line
canvas.create_line(50, 50, 350, 100, fill="blue", width=3)

# Dashed red horizontal line
canvas.create_line(50, 150, 350, 150, fill="red", width=2, dash=(4, 4))

# Multi-segment connected line (polyline)
canvas.create_line(50, 250, 150, 200, 250, 250, 350, 200, fill="green", width=3, smooth=True)

# 4. Run the application
root.mainloop()
