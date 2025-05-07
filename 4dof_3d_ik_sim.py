import matplotlib.pyplot as plt
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def circles_intersections(x0, y0, r0, x1, y1, r1):
    d = math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    if d > r0 + r1:
        return None
    if d < abs(r0 - r1):
        return None
    if d == 0 and r0 == r1:
        return None
    else:
        a = (r0 ** 2 - r1 ** 2 + d ** 2) / (2 * d)
        h = math.sqrt(r0 ** 2 - a ** 2)
        x2 = x0 + a * (x1 - x0) / d
        y2 = y0 + a * (y1 - y0) / d
        x3 = x2 + h * (y1 - y0) / d
        y3 = y2 - h * (x1 - x0) / d
        x4 = x2 - h * (y1 - y0) / d
        y4 = y2 + h * (x1 - x0) / d
        return np.array([x3, y3]), np.array([x4, y4])


def dist(pt1, pt2):
    return math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt1[1] - pt2[1]) ** 2)


def project_to_arm_plane(target, base):
    dx, dy = target[0] - base[0], target[1] - base[1]
    r = math.sqrt(dx ** 2 + dy ** 2)
    z = target[2]
    rot_angle_deg = math.degrees(math.atan2(dy, dx))
    return r, z, rot_angle_deg


class IKSolver:
    def __init__(self):
        self.lengths = None
        self.base = None

    def init_model(self, lengths, base):
        self.base = np.array(base)
        self.lengths = lengths

    def rotate_back_to_xy(self, r, z, angle_deg):
        angle_rad = math.radians(angle_deg)
        x = self.base[0] + r * math.cos(angle_rad)
        y = self.base[1] + r * math.sin(angle_rad)
        return np.array([x, y, z])

    def compute_relative_angle(self, v1, v2):
        a1 = math.atan2(v1[1], v1[0])
        a2 = math.atan2(v2[1], v2[0])
        angle_deg = math.degrees(a2 - a1)
        return ((angle_deg + 180) % 360) - 180

    def compute(self, x, y, z, end_effector_angle):
        r, z_rel, rot_angle = project_to_arm_plane((x, y, z), self.base)
        angle_rad = math.radians(end_effector_angle)
        dx = math.cos(angle_rad) * self.lengths[-1]
        dz = math.sin(angle_rad) * self.lengths[-1]
        wrist = np.array([r - dx, z_rel - dz])
        possible_joints = circles_intersections(0, 0, self.lengths[0], *wrist, self.lengths[1])
        anchor = np.array([0, 100])
        shoulder = None

        if possible_joints is not None:
            if len(possible_joints) == 2:
                shoulder = min(possible_joints, key=lambda pt: dist(pt, anchor))
            else:
                shoulder = possible_joints[0]

            shoulder_3d = self.rotate_back_to_xy(shoulder[0], shoulder[1], rot_angle)
            wrist_3d = self.rotate_back_to_xy(wrist[0], wrist[1], rot_angle)
            ee_3d = self.rotate_back_to_xy(r, z_rel, rot_angle)

            self.display_joint_angles(r, shoulder, wrist, z_rel, rot_angle)
            self.plot_3d(shoulder_3d, wrist_3d, ee_3d)

    def display_joint_angles(self, r, shoulder, wrist, z_rel, rot_angle):
        v1 = np.array(shoulder) - np.array([0, 0])
        v2 = np.array(wrist) - np.array(shoulder)
        v3 = np.array([r, z_rel]) - np.array(wrist)
        angle1 = self.compute_relative_angle([1, 0], v1)
        angle2 = self.compute_relative_angle(v1, v2)
        angle3 = self.compute_relative_angle(v2, v3)
        print(f"Base rotation (around Z axis): {rot_angle:.2f}°")
        print(f"Joint 1 angle (base to shoulder): {angle1:.2f}°")
        print(f"Joint 2 angle (shoulder to wrist): {angle2:.2f}°")
        print(f"Joint 3 angle (wrist to end-effector): {angle3:.2f}°")

    def plot_3d(self, joint, wrist, end_effector):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        base_3d = np.array([*self.base, 0])
        for pt, color, label in zip([base_3d, joint, wrist, end_effector], ['k', 'b', 'g', 'r'],
                                    ['Base', 'Shoulder', 'Wrist', 'End Effector']):
            ax.scatter(*pt, color=color, s=60, label=label)
        ax.plot([base_3d[0], joint[0]], [base_3d[1], joint[1]], [base_3d[2], joint[2]], 'b')
        ax.plot([joint[0], wrist[0]], [joint[1], wrist[1]], [joint[2], wrist[2]], 'g')
        ax.plot([wrist[0], end_effector[0]], [wrist[1], end_effector[1]], [wrist[2], end_effector[2]], 'r')
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim([-12, 12])
        ax.set_ylim([-12, 12])
        ax.set_zlim([0, 12])
        ax.view_init(elev=30, azim=60)
        ax.legend()
        plt.show()


def main():
    solver = IKSolver()
    solver.init_model([6, 5, 2], [0, 0])
    solver.compute(8, 9, 0, -45)


if __name__ == "__main__":
    main()
