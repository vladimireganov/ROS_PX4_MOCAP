import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd

def create_new_fig(num_of_subplots):
    #
    # title, xlabel, ylabel
    #
    subplots = []
    fig = plt.figure()
    spec = gridspec.GridSpec(ncols=num_of_subplots//3, nrows=num_of_subplots%3, figure=fig)
    for i in range(num_of_subplots):
       subplots.append( fig.add_subplot(spec[i//3,i%3]))
       plt.grid()

    return fig, subplots


