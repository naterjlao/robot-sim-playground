import tkinter as tk

class BouncingBallApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Tkinter Canvas Animation")
        
        # Setup Canvas
        self.canvas = tk.Canvas(root, width=400, height=300, bg="black")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Create a ball (oval) and store its ID
        self.ball = self.canvas.create_oval(10, 10, 40, 40, fill="cyan")
        
        # Initial velocity vectors (speed/direction)
        self.dx = 3
        self.dy = 2
        
        # Start the recursive animation loop
        self.animate()

    def animate(self):
        # Move the object by its dx and dy coordinates
        self.canvas.move(self.ball, self.dx, self.dy)
        
        # Fetch current coordinates of the ball [x1, y1, x2, y2]
        x1, y1, x2, y2 = self.canvas.coords(self.ball)
        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        
        # Bounce off the left or right edges
        if x1 <= 0 or x2 >= canvas_width:
            self.dx = -self.dx
            
        # Bounce off the top or bottom edges
        if y1 <= 0 or y2 >= canvas_height:
            self.dy = -self.dy
            
        # Schedule the next frame update in 16 milliseconds (~60 FPS)
        self.root.after(16, self.animate)

if __name__ == "__main__":
    root = tk.Tk()
    app = BouncingBallApp(root)
    root.mainloop()
