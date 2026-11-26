import math
import tkinter as tk
import numpy as np

class RobotSimulation:
    LENGTH = np.array([160, 120, 100])
    e0 = np.array([300, 300, 0])

    def rotate(
            e : np.ndarray, # Actuator end-effector position n=3
            l : int, # Rotated actuator length
            t : int, # Relative rotation (degrees)
        ) -> np.ndarray: # n = 3

        radians = t * math.pi/180
        r = np.array([math.cos(radians), -math.sin(radians), 0]) # Unit vector rotation from theta (t).
        return e + r*l

    def fk2(
            t : np.ndarray # theta (t) vector (n=3)
        ) -> np.ndarray: # endpoint position matrix (n=3, m=3)

        r = np.array([[1,0,0],[1,1,0],[1,1,1]]) @ t
        e1 = RobotSimulation.rotate(RobotSimulation.e0, RobotSimulation.LENGTH[0], r[0])
        e2 = RobotSimulation.rotate(e1, RobotSimulation.LENGTH[1], r[1])
        e3 = RobotSimulation.rotate(e2, RobotSimulation.LENGTH[2], r[2])

        return np.array([e1, e2, e3]).transpose()

    def __init__(self, root : tk.Tk):
        self.root = root
        self.root.title("Robot Simulator 3 DOF")

        # Setup Canvas
        self.canvas = tk.Canvas(root, width=800, height=300, bg="black")
        self.canvas.pack(fill="both", expand=True)

        # Initial Actuator Angles
        self.t = np.array([45.0,-45.0,-45.0])

        # Initial Actuator Endpoint/Setpoint Positions
        self.e = RobotSimulation.fk2(self.t)
        self.g = self.e[:,2]

        # Actuators
        self.a1 = self.canvas.create_line(0,0,0,0,width=6,fill="red")
        self.a2 = self.canvas.create_line(0,0,0,0,width=6,fill="green")
        self.a3 = self.canvas.create_line(0,0,0,0,width=6,fill="blue")

        self.canvas.bind("<Button-1>", self.on_click)

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
        init_slider(self.t[0], self.t0_update)
        init_slider(self.t[1], self.t1_update)
        init_slider(self.t[2], self.t2_update)

        self.animate()
    
    def animate(self):
        J = RobotSimulation.jacobian(self.e, self.t)
        J_inv = np.linalg.pinv(J)
        de = self.g - self.e[:,2]
        dt = J_inv @ de
        self.t = self.t + 0.1 * dt
        self.e = RobotSimulation.fk2(self.t)

        self.canvas.coords(self.a1, RobotSimulation.e0[0],  RobotSimulation.e0[1],  self.e[0,0],    self.e[1,0])
        self.canvas.coords(self.a2, self.e[0,0],            self.e[1,0],            self.e[0,1],    self.e[1,1])
        self.canvas.coords(self.a3, self.e[0,1],            self.e[1,1],            self.e[0,2],    self.e[1,2])
        self.root.after(16, self.animate)

    def t0_update(self, val):
        self.t = np.array([int(val), 0, 0]) + np.array([[0,0,0], [0,1,0], [0,0,1]]) @ self.t
        self.e = RobotSimulation.fk2(self.t)

    def t1_update(self, val):
        self.t = np.array([0, int(val), 0]) + np.array([[1,0,0], [0,0,0], [0,0,1]]) @ self.t
        self.e = RobotSimulation.fk2(self.t)

    def t2_update(self, val):
        self.t = np.array([0, 0, int(val)]) + np.array([[1,0,0], [0,1,0], [0,0,0]]) @ self.t
        self.e = RobotSimulation.fk2(self.t)

    def jacobian(e, t, dt=0.5):
        de = RobotSimulation.fk2(t + np.array([1,1,1]) * dt) - e
        return de / dt

    def on_click(self, event):
        self.g = np.array([int(event.x), int(event.y), 0])
        print(f"{self.g=}")
        # J = RobotSimulation.jacobian(self.e, self.t)
        # J_inv = np.linalg.pinv(J)

        # setpoint = np.array([int(event.x), int(event.y), 0])

        # dt = J_inv @ (self.e[:,2] - setpoint)
        # print(dt)
        # self.t += dt.astype(int)
        # self.e = RobotSimulation.fk2(self.t)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulation(root)
    root.mainloop()
