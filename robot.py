import math
import tkinter as tk
import numpy as np

class RobotSimulation:
    def rotate(
            e : np.ndarray, # Actuator end-effector position n=3
            l : int, # Rotated actuator length
            t : int, # Relative rotation (degrees)
        ) -> np.ndarray: # n = 3

        radians = t * math.pi / 180
        r = np.array([math.cos(radians), -math.sin(radians), 0]) # Unit vector rotation from theta (t).
        return e + r*l

    def fk2(
            self,
            t : np.ndarray # theta (t) vector (n=3)
        ) -> np.ndarray: # endpoint position matrix (n=3, m=3)

        r = self.R @ t
        p = self.e0
        e = []
        for idx in range(self.n):
            p = RobotSimulation.rotate(p, self.lengths[idx], r[idx])
            e += [p]

        return np.array(e).transpose()

    def __init__(self, root : tk.Tk, lengths : list[int] = [160, 120, 100, 100]):
        self.root = root
        self.n = len(lengths)
        self.lengths = lengths
        self.root.title(f"Robot Simulator {self.n} DOF")

        # Setup Canvas
        self.canvas = tk.Canvas(root, width=800, height=300, bg="black")
        self.canvas.pack(fill="both", expand=True)

        # Initial Actuator Angles
        self.t = np.zeros(self.n)
        self.dt = 0.5                   # Jacobian Estimator Delta Theta
        self.R = np.tri(self.n, self.n) # Rotational Sum Matrix

        # Initial Actuator Endpoint/Setpoint Positions
        self.e0 = np.array([300, 300, 0]) # Origin Position
        self.e = self.fk2(self.t)
        self.g = self.e[:,self.n - 1]
        self.alpha = 0.5
        self.canvas.bind("<Motion>", self.g_update)

        # Actuators
        COLOR_PATTERN = ["red", "green", "blue", "orange"]
        self.a = []
        for idx in range(self.n):
            self.a += [self.canvas.create_line(0,0,0,0, width=6, fill=COLOR_PATTERN[idx % len(COLOR_PATTERN)])]

        # Actuators Angle Sliders
        def init_slider(left, right, tickInterval, resolution, initial, command):
            s = tk.Scale(
                root,
                from_=left,                 # Minimum value (note the trailing underscore)
                to=right,                   # Maximum value
                orient=tk.HORIZONTAL,       # tk.HORIZONTAL or tk.VERTICAL
                length=200,                 # Width/height in pixels
                tickinterval=tickInterval,  # Displays numbers along the track
                resolution=resolution,      # Step increment (use 0.1 for decimals)
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
        J = self.jacobian(self.e, self.t, dt=self.dt)
        J_inv = np.linalg.pinv(J)
        de = self.g - self.e[:,self.n - 1]
        dt = J_inv @ de
        self.t = self.t + self.alpha * dt
        self.e = self.fk2(self.t)

        # Update Actuator Positions
        px = self.e0[0]
        py = self.e0[1]
        for idx in range(self.n):
            self.canvas.coords(self.a[idx], px, py, self.e[0,idx], self.e[1,idx])
            px = self.e[0,idx]
            py = self.e[1,idx]

        self.root.after(16, self.animate)

    # def t0_update(self, val):
    #     self.t = np.array([int(val), 0, 0, 0]) + np.array([[0,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]) @ self.t
    #     self.e = self.fk2(self.t)

    # def t1_update(self, val):
    #     self.t = np.array([0, int(val), 0, 0]) + np.array([[1,0,0,0], [0,0,0,0], [0,0,1,0], [0,0,0,1]]) @ self.t
    #     self.e = self.fk2(self.t)

    # def t2_update(self, val):
    #     self.t = np.array([0, 0, int(val), 0]) + np.array([[1,0,0,0], [0,1,0,0], [0,0,0,0], [0,0,0,1]]) @ self.t
    #     self.e = self.fk2(self.t)

    def dt_update(self, val):
        self.dt = float(val)

    def alpha_update(self, val):
        self.alpha = float(val)

    def jacobian(self, e, t, dt=0.5):
        de = self.fk2(t + np.ones(self.n) * dt) - e
        return de / dt

    def g_update(self, event):
        self.g = np.array([int(event.x), int(event.y), 0])

if __name__ == "__main__":
    root = tk.Tk()
    # app = RobotSimulation(root, lengths=[160, 120, 100, 100, 100, 100, 50])
    # app = RobotSimulation(root, lengths=[40]*20)
    # lengths = []
    # for idx in range(10):
    #     lengths = [(idx + 1) * 40] + lengths
    lengths = [(idx + 1) * 25 * (1 + math.sqrt(5)) / 2  for idx in range(5)][::-1]
    print(lengths)
    app = RobotSimulation(root, lengths=lengths)
    root.mainloop()
