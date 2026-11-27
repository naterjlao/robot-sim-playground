import math
import tkinter as tk
import numpy as np

class RobotSimulation:
    LENGTH = [160, 120, 100, 100]
    ROTATION_SUM_MATRIX = np.array([[1,0,0,0],[1,1,0,0],[1,1,1,0],[1,1,1,1]])
    e0 = np.array([300, 300, 0])

    def rotate(
            e : np.ndarray, # Actuator end-effector position n=3
            l : int, # Rotated actuator length
            t : int, # Relative rotation (degrees)
        ) -> np.ndarray: # n = 3

        radians = t * math.pi / 180
        r = np.array([math.cos(radians), -math.sin(radians), 0]) # Unit vector rotation from theta (t).
        return e + r*l

    def fk2(
            t : np.ndarray # theta (t) vector (n=3)
        ) -> np.ndarray: # endpoint position matrix (n=3, m=3)

        r = RobotSimulation.ROTATION_SUM_MATRIX @ t
        e1 = RobotSimulation.rotate(RobotSimulation.e0, RobotSimulation.LENGTH[0], r[0])
        e2 = RobotSimulation.rotate(e1, RobotSimulation.LENGTH[1], r[1])
        e3 = RobotSimulation.rotate(e2, RobotSimulation.LENGTH[2], r[2])
        e4 = RobotSimulation.rotate(e3, RobotSimulation.LENGTH[3], r[3])

        return np.array([e1, e2, e3, e4]).transpose()

    def __init__(self, root : tk.Tk):
        self.root = root
        self.root.title("Robot Simulator 4 DOF")

        # Setup Canvas
        self.canvas = tk.Canvas(root, width=800, height=300, bg="black")
        self.canvas.pack(fill="both", expand=True)

        # Initial Actuator Angles
        self.t = np.array([45.0,-45.0,-45.0,45.0])
        self.dt = 0.5 # Jacobian Estimator Delta Theta

        # Initial Actuator Endpoint/Setpoint Positions
        self.e = RobotSimulation.fk2(self.t)
        self.g = self.e[:,3]
        self.alpha = 0.1

        # Actuators
        self.a1 = self.canvas.create_line(0,0,0,0,width=6,fill="red")
        self.a2 = self.canvas.create_line(0,0,0,0,width=6,fill="green")
        self.a3 = self.canvas.create_line(0,0,0,0,width=6,fill="blue")
        self.a4 = self.canvas.create_line(0,0,0,0,width=6,fill="orange")

        self.canvas.bind("<Motion>", self.on_click)

        # Actuators Angle Sliders
        def init_slider(left, right, tickInterval, resolution, initial, command):
            s = tk.Scale(
                root,
                from_=left,              # Minimum value (note the trailing underscore)
                to=right,               # Maximum value
                orient=tk.HORIZONTAL, # tk.HORIZONTAL or tk.VERTICAL
                length=200,           # Width/height in pixels
                tickinterval=tickInterval,      # Displays numbers along the track
                resolution=resolution,         # Step increment (use 0.1 for decimals)
                command=command
            )
            s.set(initial)            # Set initial starting position
            s.pack(pady=10)
        # init_slider(180, -180, 90, 1, self.t[0], self.t0_update)
        # init_slider(180, -180, 90, 1, self.t[1], self.t1_update)
        # init_slider(180, -180, 90, 1, self.t[2], self.t2_update)
        init_slider(0.001, 10, 1, 0.001, self.dt, self.dt_update)
        init_slider(0.001, 3, 1, 0.001, self.alpha, self.alpha_update)

        self.animate()
    
    def animate(self):
        # Inverse Kinematics
        J = RobotSimulation.jacobian(self.e, self.t, dt=self.dt)
        J_inv = np.linalg.pinv(J)
        de = self.g - self.e[:,3]
        dt = J_inv @ de
        self.t = self.t + self.alpha * dt
        self.e = RobotSimulation.fk2(self.t)

        self.canvas.coords(self.a1, RobotSimulation.e0[0],  RobotSimulation.e0[1],  self.e[0,0],    self.e[1,0])
        self.canvas.coords(self.a2, self.e[0,0],            self.e[1,0],            self.e[0,1],    self.e[1,1])
        self.canvas.coords(self.a3, self.e[0,1],            self.e[1,1],            self.e[0,2],    self.e[1,2])
        self.canvas.coords(self.a4, self.e[0,2],            self.e[1,2],            self.e[0,3],    self.e[1,3])
        self.root.after(16, self.animate)

    def t0_update(self, val):
        self.t = np.array([int(val), 0, 0, 0]) + np.array([[0,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]) @ self.t
        self.e = RobotSimulation.fk2(self.t)

    def t1_update(self, val):
        self.t = np.array([0, int(val), 0, 0]) + np.array([[1,0,0,0], [0,0,0,0], [0,0,1,0], [0,0,0,1]]) @ self.t
        self.e = RobotSimulation.fk2(self.t)

    def t2_update(self, val):
        self.t = np.array([0, 0, int(val), 0]) + np.array([[1,0,0,0], [0,1,0,0], [0,0,0,0], [0,0,0,1]]) @ self.t
        self.e = RobotSimulation.fk2(self.t)

    def dt_update(self, val):
        self.dt = float(val)

    def alpha_update(self, val):
        self.alpha = float(val)

    def jacobian(e, t, dt=0.5):
        de = RobotSimulation.fk2(t + np.ones(4) * dt) - e
        return de / dt

    def on_click(self, event):
        self.g = np.array([int(event.x), int(event.y), 0])

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulation(root)
    root.mainloop()
