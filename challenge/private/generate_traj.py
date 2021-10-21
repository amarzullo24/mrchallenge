import numpy as np
import matplotlib.pyplot as plt
def func(x):
    return np.sin(x)

def main():
    n_points = 100
    x_vals = np.linspace(0,3*np.pi,n_points)
    for i in x_vals:
        x = func(i) / 10
        y = 0 #func(i) / 10
        z = 0 #func(i) / 10

        with open('trajectory.txt', 'a') as fout:
            fout.write(str((x,y,z)) + '\n')

    #plt.plot(x_vals, np.sin(x_vals), '-')
    #plt.show()
if __name__ == "__main__":
    main()