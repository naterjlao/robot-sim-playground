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

    LENGTH = (160, 120, 100)

    def __init__(self, root):
        self.root = root
        self.root.title("Robot Simulator 3 DOF")

        # Setup Canvas
        self.canvas = tk.Canvas(root, width=800, height=300, bg="black")
        self.canvas.pack(fill="both", expand=True)

        # Actuators
        self.a1 = self.canvas.create_line(0,0,0,0,fill="red", width=6)
        self.a2 = self.canvas.create_line(0,0,0,0,fill="green", width=6)
        self.a3 = self.canvas.create_line(0,0,0,0,fill="blue", width=6)

        # Origin and Initial Actuator Endpoint Position
        self.p0 = (300, 300)
        self.p1 = (0, 0)
        self.p2 = (0, 0)
        self.p3 = (0, 0)

        # Initial Actuator Angles
        self.t1 = 45
        self.t2 = -45
        self.t3 = -45

        # Actuators Angle Sliders
        def init_slider(initial, command):
            s = tk.Scale(
                root,
                from_=180,              # Minimum value (note the trailing underscore)
                to=-180,               # Maximum value
                orient=tk.HORIZONTAL, # tk.HORIZONTAL or tk.VERTICAL
                length=200,           # Width/height in pixels
                tickinterval=90,      # Displays numbers along the track
                resolution=1,         # Step increment (use 0.1 for decimals)
                command=command
            )
            s.set(initial)            # Set initial starting position
            s.pack(pady=10)
        init_slider(self.t1, self.t1_update)
        init_slider(self.t2, self.t2_update)
        init_slider(self.t3, self.t3_update)

        self.animate()
    
    def animate(self):
        self.p1 = RobotSimulation.rotate(self.p0, self.LENGTH[0], self.t1)
        self.p2 = RobotSimulation.rotate(self.p1, self.LENGTH[1], self.t1 + self.t2)
        self.p3 = RobotSimulation.rotate(self.p2, self.LENGTH[2], self.t1 + self.t2 + self.t3)
        self.canvas.coords(self.a1, self.p0[0], self.p0[1], self.p1[0], self.p1[1])
        self.canvas.coords(self.a2, self.p1[0], self.p1[1], self.p2[0], self.p2[1])
        self.canvas.coords(self.a3, self.p2[0], self.p2[1], self.p3[0], self.p3[1])
        self.root.after(16, self.animate)

    def t1_update(self, val): self.t1 = int(val)
    def t2_update(self, val): self.t2 = int(val)
    def t3_update(self, val): self.t3 = int(val)

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulation(root)
    root.mainloop()
