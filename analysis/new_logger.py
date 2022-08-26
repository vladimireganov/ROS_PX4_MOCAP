import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd

from plot_some_data import plot_data

# def read_data(file):
    # return pd.read_csv(file,sep=",")

# path = "analysis/logs/square/square5"


import sys, getopt

# default_output = "test.csv"

def main(argv):
    inputfile = ''
    # outputfile = 'test.csv'
    try:
        opts, args = getopt.getopt(argv,"hi:",["ifile="])
    except getopt.GetoptError:
        print ('new_logger.py -i <inputfile>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print ('new_logger.py -i <inputfile>')
            sys.exit()
        elif opt in ("-i", "--ifile"):
            inputfile = arg
        # elif opt in ("-o","--ofile"):
        #     outputfile = arg
        else:
            print ('Input file was not specified')
            sys.exit()
    print ('Input file is :', inputfile)
    # print ('Output file is :', outputfile)
    plot_data(inputfile)


if __name__ == "__main__":
   main(sys.argv[1:])