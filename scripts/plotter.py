import numpy as np
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D



def plotter(csv_file_name="poses.csv"):
    df = pd.read_csv("../csv_output/" + csv_file_name, header=None)
    # df = pd.read_csv("/home/dyuman/Desktop/" + csv_file_name, header=None)
    x = list(df.iloc[:, 0])
    y = list(df.iloc[:, 1])
    z = list(df.iloc[:, 2])

    # x, y,z= np.meshgrid(x,y,z)
 

    # fig = plt.figure()
    fig = plt.figure(figsize=(15,10))
    # ax = fig.add_subplot(441, projection = '3d')
    ax = plt.axes(projection='3d')

    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.scatter(z, y, x)
    plt.show()




def main():
    sns.set()
    plotter()



if __name__ == "__main__":
    main()
