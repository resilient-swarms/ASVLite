import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file into a DataFrame
df = pd.read_csv("./thrust_tuning_factors.csv")

# Define wave height bins and labels
bins = [ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, float('inf')]
labels = ["0-1.0", "1.0-2.0", "2.0-3.0", "3.0-4.0", "4.0-5.0", "5.0-6.0", "6.0-7.0", "7.0-8.0", "8.0+"]

# Assign wave height bins
df["wave_ht_group"] = pd.cut(df["wave_ht"], bins=bins, labels=labels, right=False)

# Group by wave height bins and compute the mean tuning factor
grouped_df = df.groupby("wave_ht_group")["tuning_factor"].mean().reset_index()

# Plot
plt.figure(figsize=(8, 6))
bars = plt.bar(grouped_df["wave_ht_group"], grouped_df["tuning_factor"], color='b', alpha=0.7)

# Add text labels on top of each bar
for bar, value in zip(bars, grouped_df["tuning_factor"]):
    plt.text(bar.get_x() + bar.get_width() / 2, bar.get_height(), f'{value:.2f}', 
             ha='center', va='bottom', fontsize=10, fontweight='bold')

plt.xlabel("Wave Height Range")
plt.ylabel("Mean Tuning Factor")
plt.title("Wave Height Range vs Mean Tuning Factor")
plt.xticks(rotation=45)
plt.grid(axis='y')
# plt.show()
plt.tight_layout()
plt.savefig("thrust_tuning_factor.png")
