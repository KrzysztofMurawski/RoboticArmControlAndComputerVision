import matplotlib.pyplot as plt
import math
import numpy as np


base = (0, 0)
end_goal = (4, 5)

l1, l2, l3 = 5, 5, 2
a1, a2, a3 = 90, 45, 30


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


class Segment:
    def __init__(self, x, y, angle, length):
        self.start_x = x
        self.start_y = y
        self.angle = angle
        self.length = length

        delta_x = math.sin(math.radians(self.angle)) * self.length
        delta_y = math.cos(math.radians(self.angle)) * self.length


        self.end_x, self.end_y = self.start_x+delta_x, self.start_y+delta_y



class FabrikSolver:
    def __init__(self, base_x, base_y):
        self.base_x = base_x
        self.base_y = base_y

        self.segments = []

    def add_segment(self, angle, length):
        if len(self.segments) > 0:
            self.segments.append(Segment(
                self.segments[-1].end_x,
                self.segments[-1].end_y, angle + self.segments[-1].angle, length))
        else:
            self.segments.append(Segment(
                self.base_x,
                self.base_y, angle, length))

    def compute(self, x, y):
        target = np.array([x, y])

        target_to_start_vector = np.array([self.segments[-1].start_x - target[0], self.segments[-1].start_y - target[1]])
        target_to_start_vector_length_corr = unit_vector(target_to_start_vector) * self.segments[-1].length

        self.segments[-1].start_x = target[0]
        self.segments[-1].start_y = target[1]
        self.segments[-1].end_x = target[0] + target_to_start_vector_length_corr[0]
        self.segments[-1].end_y = target[1] + target_to_start_vector_length_corr[1]

        for i in range(len(self.segments)-2, -1, -1):

            target_to_start_vector = np.array(
                [self.segments[i].start_x - self.segments[i+1].end_x,
                 self.segments[i].start_y - self.segments[i+1].end_y])
            target_to_start_vector_length_corr = unit_vector(target_to_start_vector) * self.segments[i].length

            self.segments[i].start_x = self.segments[i+1].end_x
            self.segments[i].start_y = self.segments[i+1].end_y
            self.segments[i].end_x = self.segments[i+1].end_x + target_to_start_vector_length_corr[0]
            self.segments[i].end_y = self.segments[i+1].end_y + target_to_start_vector_length_corr[1]

        # FOREWARDS

        base = np.array([self.base_x, self.base_y])

        base_to_end_vector = np.array([self.segments[0].end_x - base[0], self.segments[0].end_y - base[1]])
        base_to_end_vector_length_corr = unit_vector(base_to_end_vector) * self.segments[0].length

        self.segments[0].start_x = base[0]
        self.segments[0].start_y = base[1]
        self.segments[0].end_x = base[0] + base_to_end_vector_length_corr[0]
        self.segments[0].end_y = base[1] + base_to_end_vector_length_corr[1]

        for i in range(1, len(self.segments)):

            target_to_start_vector = np.array(
                [self.segments[i].start_x - self.segments[i-1].end_x,
                 self.segments[i].start_y - self.segments[i-1].end_y])
            target_to_start_vector_length_corr = unit_vector(target_to_start_vector) * self.segments[i].length

            self.segments[i].start_x = self.segments[i-1].end_x
            self.segments[i].start_y = self.segments[i-1].end_y
            self.segments[i].end_x = self.segments[i-1].end_x + target_to_start_vector_length_corr[0]
            self.segments[i].end_y = self.segments[i-1].end_y + target_to_start_vector_length_corr[1]




if __name__ == "__main__":

    solver = FabrikSolver(0, 0)

    solver.add_segment(10, 5)
    solver.add_segment(60, 5)
    solver.add_segment(60, 2)


    plt.xlim([-10, 10])
    plt.ylim([-10, 10])

    for segment in solver.segments:
        plt.plot([segment.start_x,  segment.end_x], [segment.start_y,segment.end_y], marker = "o", c="b")

    target = (2, 3)
    accuracy = 0.001
    while abs(target[0]-solver.segments[-1].end_x) > accuracy and abs(target[1]-solver.segments[-1].end_y) > accuracy:
        solver.compute(*target)


    for segment in solver.segments:
        plt.plot([segment.start_x,  segment.end_x], [segment.start_y,segment.end_y], marker = "o", c="r")

    plt.plot(2, 3, c="g", marker="o")
    plt.show()
