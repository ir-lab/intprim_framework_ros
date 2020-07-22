#   @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt

def hinton_fast(W):
    plt.figure()
    plt.imshow(W, cmap='gray', interpolation='none')
    plt.colorbar()
    plt.show()
