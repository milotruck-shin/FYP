import pandas as pd
import matplotlib.pyplot as plt

# ============================================
# CSV to Line Plot - Simple Example
# ============================================

# Step 1: Read the CSV file
# Replace 'data.csv' with your actual file path
csv_file = r"D:\motor logs\no load dataset 1.csv"

def extract_numeric(value):
    """Extract numeric value from strings with units (e.g., '0.600 rev' -> 0.6)"""
    if isinstance(value, str):
        # Remove all non-numeric characters except decimal point and minus sign
        import re
        match = re.search(r'-?\d+\.?\d*', value.strip())
        if match:
            return float(match.group())
    return float(value)

try:
    df = pd.read_csv(csv_file)
    print(f"Successfully loaded CSV file with {len(df)} rows")
    print(f"Columns: {df.columns.tolist()}")
except FileNotFoundError:
    print(f"Error: File '{csv_file}' not found.")
    exit()

# Step 2: Create the plot

time_s = df.iloc[:, 0].apply(extract_numeric)
command_pos = df.iloc[:, 2].apply(extract_numeric)
actual_pos = df.iloc[:, 3].apply(extract_numeric)

time_relative = time_s - time_s.iloc[0]


plt.figure(figsize=(12, 6))
 
# Plot both columns against time
plt.plot(time_relative, command_pos, marker='o', linewidth=2, label='Commanded Pos (Rev)', markersize=2)
plt.plot(time_relative, actual_pos, marker='s', linewidth=2, label='Motor Pos (Rev)', markersize=2)
 
# Step 4: Customize the plot
plt.xlabel('Time (seconds)', fontsize=12)
plt.ylabel('Revolution (rev)', fontsize=12)
plt.title('Commanded Pos & Motor Pos vs Time - D1', fontsize=14, fontweight='bold')
plt.legend(loc='best', fontsize=10)
plt.grid(True, alpha=0.3)
plt.tight_layout()
 
# Step 5: Display or save the plot
plt.show()  # Display the plot in a window
# plt.savefig('output_plot.png', dpi=300)  # Uncomment to save as image
 
# Print data info
print(f"\nData summary:")
print(f"Time range: {time_relative.min():.2f}s to {time_relative.max():.2f}s")
print(f"Column 3 - Min: {command_pos.min():.4f}, Max: {command_pos.max():.4f}")
print(f"Column 4 - Min: {actual_pos.min():.4f}, Max: {actual_pos.max():.4f}")