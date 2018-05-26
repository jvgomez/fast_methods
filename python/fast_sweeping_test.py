import numpy as np
import matplotlib.pyplot as plt
import imageio
import os


class FastSweeping(object):
    '''
    Assume 2D. Assume that grid is the velocities map. Assume that cell size is 1x1
    '''
    def __init__(self, grid, init_point):
        self._W = grid
        self._init = init_point

        self._T = np.full(self._W.shape, np.inf)
        self._T[self._init[0], self._init[1]] = 0

    # Original version
    def _get_sweep_ranges(self, n):
        sweep_dir = n % 4

        if sweep_dir == 0:
            x_dir = 1
            y_dir = 1
        elif sweep_dir == 1:
            x_dir = 1
            y_dir = -1
        elif sweep_dir == 2:
            x_dir = -1
            y_dir = -1
        elif sweep_dir == 3:
            x_dir = -1
            y_dir = 1

        if x_dir == 1:
            x_range = range(0, self._T.shape[0])
        else:
            x_range = range(self._T.shape[0] - 1, -1, -1)
        if y_dir == 1:
            y_range = range(0, self._T.shape[1])
        else:
            y_range = range(self._T.shape[1] - 1, -1, -1)
        return x_range, y_range

    # Paper version
    def _get_sweep_ranges_2(self, n):
        sweep_dir = n % 4

        if sweep_dir == 0:
            x_dir = -1
            y_dir = -1
        elif sweep_dir == 1:
            x_dir = 1
            y_dir = -1
        elif sweep_dir == 2:
            x_dir = -1
            y_dir = 1
        elif sweep_dir == 3:
            x_dir = 1
            y_dir = 1

        if x_dir == 1:
            x_range = range(0, self._T.shape[0])
        else:
            x_range = range(self._T.shape[0] - 1, -1, -1)
        if y_dir == 1:
            y_range = range(0, self._T.shape[1])
        else:
            y_range = range(self._T.shape[1] - 1, -1, -1)
        return x_range, y_range

    def _solve_eikonal(self, x,y):
        if self._W[x, y] == 0:
            return np.inf

        x_vals = []
        y_vals = []
        if x - 1 >= 0:
            x_vals.append(self._T[x - 1, y])
        if x + 1 < self._T.shape[0]:
            x_vals.append(self._T[x + 1, y])
        if y - 1 >= 0:
            y_vals.append(self._T[x, y - 1])
        if y + 1 < self._T.shape[1]:
            y_vals.append(self._T[x, y + 1])
        
        Tx = min(x_vals)
        Ty = min(y_vals)

        if np.isinf(Tx) and np.isinf(Ty):
            return np.inf

        # Checking 2D causality
        if abs(Tx - Ty) > 1 / self._W[x, y]:
            return min(Tx, Ty) + 1 / self._W[x, y] # 1D update

        a = 2
        b = -2 * (Tx + Ty)
        c = Tx**2 + Ty**2 - 1 / self._W[x, y]**2
        quad_term = b**2 - 4 * a * c

        if quad_term < 0:
            return np.inf
        else:
            return (-b + np.sqrt(quad_term)) / (2 * a)


    def plot_T(self):
        plt.matshow(self._T, cmap='plasma')
        plt.show()

    def solve(self):
        n_sweeps = 0
        stop = False

        while not stop:
            x_range, y_range = self._get_sweep_ranges(n_sweeps)
            #x_range, y_range = self._get_sweep_ranges_2(n_sweeps)
            stop = True

            for x in x_range:
                for y in y_range:
                    t = self._solve_eikonal(x,y)
                    if not np.isinf(t) and t < self._T[x, y]:
                        self._T[x, y] = t
                        stop = False

            n_sweeps += 1

        return n_sweeps


if __name__ == '__main__':
    current_dir = os.path.dirname(__file__)
    image_file = os.path.join(current_dir, '..', 'data','easy.png')
    grid = imageio.imread(image_file)
    grid = grid / 255 # Hack to have velocities in the range 0-1
    #grid = np.full((50, 100), 1) # Empty grid to test
    init_point = map(int, (0.5 * grid.shape[0], 0.5 * grid.shape[1]))

    fsm = FastSweeping(grid, init_point)
    n_sweeps = fsm.solve()

    print n_sweeps

    fsm.plot_T()