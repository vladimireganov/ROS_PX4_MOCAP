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


import sys, getopt

default_output = "test.csv"

def main(argv):
    inputfile = ''
    outputfile = 'test.csv'
    try:
        opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    except getopt.GetoptError:
        print ('test.py -i <inputfile> -o <outputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('test.py -i <inputfile> -o <outputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        # elif opt in ("-o","--ofile"):
        #     outputfile = arg
        else:
            print ('Input file was not specified')
            sys.exit()
    print ('Input file is :', inputfile)
    print ('Output file is :', outputfile)


if __name__ == "__main__":
   main(sys.argv[1:])