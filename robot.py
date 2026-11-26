import math
import tkinter as tk

class RobotSimulation:
    def rotate(
            origin : tuple[int],
            length : int,
            theta : int # degrees
        ) -> tuple[int]:
        dx = math.cos(theta * math.pi / 180) * length
        dy = math.sin(theta * math.pi / 180) * length
        return round(origin[0] + dx), round(origin[1] - dy)

    ORIGIN = (150, 150)
    LENGTH = (60, 40, 18 * 4)
    THETAS = (45, 0, -45)

    def __init__(self, root):
        self.root = root
        self.root.title("Robot Simulator 3 DOF")

        # Setup Canvas
        self.canvas = tk.Canvas(root, width=400, height=300, bg="black")
        self.canvas.pack(fill="both", expand=True)

        # Positions and Actuators
        self.p1 = RobotSimulation.rotate(
            self.ORIGIN,
            self.LENGTH[0],
            self.THETAS[0])
        self.a1 = self.canvas.create_line(
            self.ORIGIN[0],
            self.ORIGIN[1],
            self.ORIGIN[0],
            self.ORIGIN[1],
            fill="red", width=3
        )

        # Actuators Angle Setpoints (Theta)
        s = tk.Scale(
            root,
            from_=180,              # Minimum value (note the trailing underscore)
            to=-180,               # Maximum value
            orient=tk.HORIZONTAL, # tk.HORIZONTAL or tk.VERTICAL
            length=200,           # Width/height in pixels
            tickinterval=90,      # Displays numbers along the track
            resolution=1,         # Step increment (use 0.1 for decimals)
            command=self.s1_update
        )
        s.set(self.THETAS[0])            # Set initial starting position
        s.pack(pady=10)

        self.animate()
    
    def animate(self):
        # self.canvas.move(self.a1, 0, 0) # this forces the window to render
        self.canvas.coords(self.a1, self.ORIGIN[0], self.ORIGIN[1], self.p1[0], self.p1[1])
        self.root.after(16, self.animate)

    def s1_update(self,val):
        print(val)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulation(root)
    root.mainloop()
