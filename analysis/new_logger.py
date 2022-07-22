import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd

def read_data(file):
    return pd.read_csv(file,sep=",")

path = "analysis/logs/square/square5"

local_position = read_data(path +"_vehicle_local_position_0.csv")
set_point_position = read_data(path +"_vehicle_local_position_setpoint_0.csv")
estimator_position = read_data(path +"_estimator_local_position_0.csv")
estimator_position2 = read_data(path +"_estimator_local_position_1.csv")
visual_position = read_data(path +"_estimator_visual_odometry_aligned_0.csv")

