import pandas as pd
from datetime import datetime

class Storm_track:
    def __init__(self, file_path):
        df = pd.read_csv(file_path, delim_whitespace=True)
        df.set_axis(["year", "month", "day", "hh", "mm", "lat", "long"], axis=1, inplace=True)
        self.track = []
        for index, row in df.iterrows():
            time = datetime(int(row["year"]), int(row["month"]), int(row["day"]), int(row["hh"]), int(row["mm"]))
            self.track.append([time, row["lat"], row["long"]])
    
    def get_eye_location(self, date_time):
        index = min(range(len(self.track)), key=lambda i: abs((self.track[i][0]-date_time).total_seconds()))
        return (self.track[index][1], self.track[index][2])
