import pandas as pd
import matplotlib.pyplot as plt
from geopy.distance import geodesic

# Load and preprocess data
df = pd.read_csv("./wave_glider_onboard_data_unfiltered.csv")
df['Timestamp(UTC)'] = pd.to_datetime(df['Timestamp(UTC)'])
df['t1'] = df['Timestamp(UTC)']
df['t2'] = df['Timestamp(UTC)'].shift(-1)
df['x1'] = df[' Longitude']
df['y1'] = df[' Latitude']
df['x2'] = df[' Longitude'].shift(-1)
df['y2'] = df[' Latitude'].shift(-1)
df["delta_t(sec)"] = (df["t2"] - df["t1"]).dt.total_seconds()
df = df.dropna()

# Calculate distance and speed
def calc_distance(lat1, lon1, lat2, lon2):
    return geodesic((lat1, lon1), (lat2, lon2)).meters

df["delta_d(m)"] = df.apply(lambda row: calc_distance(row["y1"], row["x1"], row["y2"], row["x2"]), axis=1)
df["avg_speed(m/s)"] = df["delta_d(m)"] / df["delta_t(sec)"]

# Function to remove outliers using IQR method
def remove_outliers(df, column):
    Q1 = df[column].quantile(0.25)  
    Q3 = df[column].quantile(0.75) 
    return df[(df[column] >= Q1) & (df[column] <= Q3)]

# Clean data
df_cleaned = remove_outliers(df, "avg_speed(m/s)")
df_cleaned.to_csv("wave_glider_onboard_data_filtered.csv", index=False)

# Create box plots before and after refinement
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.boxplot(df["avg_speed(m/s)"].dropna(), vert=True, patch_artist=True)
plt.title("Before Refinement")
plt.ylabel("Speed (m/s)")

plt.subplot(1, 2, 2)
plt.boxplot(df_cleaned["avg_speed(m/s)"].dropna(), vert=True, patch_artist=True)
plt.title("After Refinement")

plt.tight_layout()
plt.savefig("speed_ranges_in_data.png")
