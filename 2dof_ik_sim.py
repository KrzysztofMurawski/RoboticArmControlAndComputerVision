import matplotlib.pyplot as plt
import math
import numpy as np


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
    return math.sqrt((pt2[0] - pt1[0])**2 + (pt1[1]-pt2[1])**2)


class IKSolver:
    def __init__(self):
        self.lengths = None
        self.base = None

    def init_model(self, lengths, base):
        self.base = np.array(base)
        self.lengths = lengths

    def compute(self, end_effector_x, end_effector_y, end_effector_angle):
        x = math.cos(math.radians(end_effector_angle)) * self.lengths[-1]
        y = math.sin(math.radians(end_effector_angle)) * self.lengths[-1]

        wrist = np.array([end_effector_x - x, end_effector_y - y, 0])

        possible_joints = circles_intersections(*self.base, self.lengths[0], wrist[0], wrist[1], self.lengths[1])

        anchor = np.array([0, 100])
        shoulder = None

        if possible_joints is not None:
            if len(possible_joints) == 2:
                if dist(possible_joints[0], anchor) > dist(possible_joints[1], anchor):
                    shoulder = possible_joints[1]
                else:
                    shoulder = possible_joints[0]
            elif len(possible_joints) == 1:
                shoulder = possible_joints[0]
            self.plot(shoulder, wrist, [end_effector_x, end_effector_y])

    def plot(self, joint, target, end_effector):

        plt.xlim([-12, 12])
        plt.ylim([-1, 10])
        plt.plot(*self.base, marker="o")
        plt.plot(*target, marker="o")
        plt.plot([self.base[0], joint[0]], [self.base[1], joint[1]], marker="o")
        plt.plot([target[0], joint[0]], [target[1], joint[1]], marker="o")
        plt.plot([target[0],end_effector[0]], [target[1], end_effector[1]], marker="o")
        plt.show()


def main():
    solver = IKSolver()
    solver.init_model([6, 5, 2], [0, 1])

    solver.compute(5, 8, -90)


if __name__ == "__main__":
    main()
