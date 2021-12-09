import pandas as pd
import numpy as np
from datetime import datetime
import math
import copy

class Storm_track:
    def __init__(self, file_path):
        df = pd.read_csv(file_path, delim_whitespace=True)
        df.set_axis(["time", "predicted_track"], axis=1, inplace=True)
        self.actual_track = []
        self.predicted_track = []
        for index, row in df.iterrows():
            time = [int(item) for item in row["time"].split(",")]
            time = datetime(year=time[0], month=time[1], day=time[2], hour=time[3], minute=time[4])
            track = []
            for point in row["predicted_track"].split(";"):
                point = tuple(float(item) for item in point.strip("( )").split(","))
                track.append(point)
            self.actual_track.append([time, track[0]])
            self.predicted_track.append([time, track])
    
    def get_eye_location(self, date_time):
        index = min(range(len(self.actual_track)), key=lambda i: abs((self.actual_track[i][0]-date_time).total_seconds()))
        eye_location = self.actual_track[index][1]
        return eye_location

    def get_projected_point_on_extrapolated_track(self, position, date_time):
        index = min(range(len(self.actual_track)), key=lambda i: abs((self.actual_track[i][0]-date_time).total_seconds()))
        predicted_track = copy.deepcopy(self.predicted_track[index][1])
        # Find the projection of the point on the last line segment of the track.
        # Create a line segment connecting last two points on the track
        i = len(predicted_track) - 2
        x_start = predicted_track[i][1]   # longitude
        y_start = predicted_track[i][0]   # latitude
        x_end   = predicted_track[i+1][1] # longitude
        y_end   = predicted_track[i+1][0] # latitude
        # Find the projection of position on the line:
        # Slope of the line passing through start and end point
        m1 = (y_end - y_start) / (x_end - x_start) if (x_end - x_start) != 0.0 else math.tan(math.pi/2)
        # Slope of the line perpendicular to the above line. Because m1*m2 = -1 for perpendicular lines.
        m2 = -1/m1 if m1 != 0.0 else -math.tan(math.pi/2) 
        c1 = y_end - (m1 * x_end)
        c2 = position[0] - (m2 * position[1])
        # The intersection point of the two lines is the projection of the given point on the line.
        # Intersection points of lines y = m1.x + c1 and y = m2.x + c2 are:
        # x = longitude = (c2 - c1) / (m1 - m2)
        # y = latitude  = (m2.c1 - m1.c2) / (m2 - m1)
        latitude_projected  = (m2*c1 - m1*c2) / (m2 - m1)
        longitude_projected = (c2 - c1) / (m1 - m2)
        predicted_track.append((latitude_projected, longitude_projected))  
        # Find the nearest projected point
        distances = []
        for point in predicted_track:
            delta_lat = position[0] - point[0]
            delta_long = position[1] - point[1]
            distance = math.sqrt(delta_lat**2 + delta_long**2)
            distances.append(distance)
        # Index of the nearest point 
        index = np.argmin(distances)
        return predicted_track[index] # returns (latitude, longitude)

        
# temp = Storm_track("./sample_files/katrina/track.csv")
# temp.get_eye_location(datetime(2005,8,29,9,0))
# print(temp.get_projected_point_on_extrapolated_track((20.8,-70.6), datetime(2005,8,29,9,0)))