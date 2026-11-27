import math
import tkinter as tk
import numpy as np

class RobotSimulation:
    def rotate(
            e : np.ndarray, # Actuator Origin Position Vector
            l : int, # Actuator Length
            t : int, # Actuator Relative Rotation (Angular Degrees)
        ) -> np.ndarray: # n = 3

        radians = t * math.pi / 180 # Covert to radians
        r = np.array([math.cos(radians), -math.sin(radians), 0]) # Unit vector rotation of theta.
        return e + r*l # Scale the rotation unit vector and displace by origin position vector.

    def forward_kinematics(
            self,
            t : np.ndarray  # Local Actuator Rotation Vector (theta, Angular Degrees)
        ) -> np.ndarray:    # Endpoint Position Matrix: (3, n)
        """Forward Kinematics Evaluator"""
        r = self.R @ t  # Compute rotation angles in respect to the origin.
        p = self.e0     # Starting initial position. Updated to the end of actuator per rotation transform
        e = []
        for idx in range(self.n):
            # Rotate each actuator starting from the origin to the end effector.
            p = RobotSimulation.rotate(p, self.lengths[idx], r[idx])
            e += [p]
        return np.array(e).transpose()

    def __init__(self, root : tk.Tk, lengths : list[int] = [160, 120, 100, 100]):
        self.root       = root
        self.lengths    = lengths
        self.n          = len(self.lengths)

        # Setup GUI
        self.root.title(f"Robot Simulator {self.n} DOF")
        self.canvas = tk.Canvas(root, width=800, height=300, bg="black")
        self.canvas.pack(fill="both", expand=True)

        # Initial Actuator Angles
        self.t = np.zeros(self.n)
        self.dt = 0.5                   # Initial Jacobian Estimator Theta Delta
        self.R = np.tri(self.n, self.n) # Rotational Sum Matrix used for forward_kinematics().

        # Initial Actuator Endpoint/Setpoint Positions
        self.e0 = np.array([300, 300, 0])           # Set robot origin position.
        self.e = self.forward_kinematics(self.t)    # Calculate initial actuator pose.
        self.g = self.e[:,self.n - 1]               # Set initial end effector setpoint.
        self.alpha = 0.5                            # Initial alpha constant for inverse_kinematics().
        self.canvas.bind("<Motion>", self.g_update) # Interactive GUI - track mouse positions.

        # Create Actuator Lines
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
        init_slider(0.5, 90, 10, 0.5, self.dt, self.dt_update)
        init_slider(0.001, 3, 1, 0.001, self.alpha, self.alpha_update)

        self.animate()
    
    def animate(self):
        # Inverse Kinematics TODO move to function
        J = self.jacobian(self.e, self.t, dt=self.dt)   # Compute (estimate) the Jacobian.
        J_inv = np.linalg.pinv(J)                       # Compute Jacobian pseudo-inverse.
        de = self.g - self.e[:,self.n - 1]              # Compute position delta between current end effector position and setpoint.
        dt = J_inv @ de                                 # Compute actuator angles delta.
        self.t = self.t + self.alpha * dt               # Add the actuator angles delta to current angles (with alpha step-size).
        self.e = self.forward_kinematics(self.t)        # Update the pose of the robot.

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
    #     self.e = self.forward_kinematics(self.t)

    # def t1_update(self, val):
    #     self.t = np.array([0, int(val), 0, 0]) + np.array([[1,0,0,0], [0,0,0,0], [0,0,1,0], [0,0,0,1]]) @ self.t
    #     self.e = self.forward_kinematics(self.t)

    # def t2_update(self, val):
    #     self.t = np.array([0, 0, int(val), 0]) + np.array([[1,0,0,0], [0,1,0,0], [0,0,0,0], [0,0,0,1]]) @ self.t
    #     self.e = self.forward_kinematics(self.t)

    def dt_update(self, val):
        self.dt = float(val)

    def alpha_update(self, val):
        self.alpha = float(val)

    def jacobian(self, e, t, dt=0.5):
        """Jacobian Estimator
        
        Computes the jacobian matrix (de/d_{theta}) using numerical estimation.
        This method "nudges" the current angular values of the actuators to compute
        a delta in the resulting position.

        e: current actuator position matrix
        t: current actuator angle values vector
        dt: "nudge" value applied to all angle values

        Returns: the jacobian matrix representing the linear rate of change
        of the actuator positions based on angular values.
        """
        de = self.forward_kinematics(t + np.ones(self.n) * dt) - e
        return de / dt

    def g_update(self, event):
        self.g = np.array([int(event.x), int(event.y), 0])

if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulation(root)
    # To setup an arbitrary number of actuators replace above with this line:
    # app = RobotSimulation(root, lengths=[160, 120, 100, 100, 100, 100, 50])
    root.mainloop()
