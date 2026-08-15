import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ============================================
# Transient Response Analysis - CSV Data
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
    # Read CSV without assuming there's a header row
    df = pd.read_csv(csv_file, header=None)
    print(f"Successfully loaded CSV file with {len(df)} rows and {len(df.columns)} columns")
    print(f"First row: {df.iloc[0].tolist()}")
except FileNotFoundError:
    print(f"Error: File '{csv_file}' not found.")
    exit()

# Step 2: Extract time and data columns using iloc (integer location)
# Column 0: Time (in microseconds)
# Column 2 (3rd column): First data series
# Column 3 (4th column): Second data series

# Extract and convert values using iloc
time_raw = df.iloc[:, 0].apply(extract_numeric)
col_3_raw = df.iloc[:, 2].apply(extract_numeric)
col_4_raw = df.iloc[:, 3].apply(extract_numeric)

# Convert time from microseconds to seconds
time_s = time_raw / 1_000_000

# Convert to relative time (start from 0)
time_relative = time_s - time_s.iloc[0]

time_us = time_raw
col_3 = col_3_raw
col_4 = col_4_raw

# Step 3: Calculate transient response metrics
def calculate_transient_metrics(time, setpoint, response, settling_tolerance=0.02):
    """
    Calculate transient response metrics
    setpoint: desired reference signal
    response: actual system response
    settling_tolerance: 0.02 for 2%, 0.05 for 5%, etc.
    """
    
    metrics = {}
    
    # Target/Steady-state value (final setpoint value)
    target_value = setpoint.iloc[-1]
    ss_response = response.iloc[-1]
    metrics['target_value'] = target_value
    metrics['steady_state'] = ss_response
    
    # Steady-state error
    metrics['ss_error'] = abs(ss_response - target_value)
    
    # Initial values
    initial_setpoint = setpoint.iloc[0]
    initial_response = response.iloc[0]
    metrics['initial_response'] = initial_response
    
    # Peak value and peak time (relative to initial response)
    peak_idx = response.idxmax() if ss_response > initial_response else response.idxmin()
    peak_value = response.iloc[peak_idx]
    peak_time = time.iloc[peak_idx]
    metrics['peak_value'] = peak_value
    metrics['peak_time'] = peak_time
    
    # Overshoot percentage (relative to setpoint change)
    setpoint_change = target_value - initial_setpoint
    if setpoint_change != 0:
        overshoot_pct = abs(peak_value - target_value) / abs(setpoint_change) * 100
    else:
        overshoot_pct = 0
    metrics['overshoot_pct'] = overshoot_pct
    
    # Rise time (10% to 90% of setpoint change)
    if setpoint_change > 0:
        rise_10 = initial_response + 0.1 * setpoint_change
        rise_90 = initial_response + 0.9 * setpoint_change
        idx_10 = np.where(response >= rise_10)[0]
        idx_90 = np.where(response >= rise_90)[0]
    else:
        rise_10 = initial_response + 0.1 * setpoint_change
        rise_90 = initial_response + 0.9 * setpoint_change
        idx_10 = np.where(response <= rise_10)[0]
        idx_90 = np.where(response <= rise_90)[0]
    
    if len(idx_10) > 0 and len(idx_90) > 0:
        rise_time = time.iloc[idx_90[0]] - time.iloc[idx_10[0]]
        metrics['rise_time'] = rise_time
        metrics['rise_time_10'] = time.iloc[idx_10[0]]
        metrics['rise_time_90'] = time.iloc[idx_90[0]]
    else:
        metrics['rise_time'] = None
    
    # Settling time (2% criterion relative to setpoint)
    settling_band_upper = target_value + settling_tolerance * abs(setpoint_change)
    settling_band_lower = target_value - settling_tolerance * abs(setpoint_change)
    
    # Find where signal enters and stays in the band
    if setpoint_change > 0:
        in_band = (response >= settling_band_lower) & (response <= settling_band_upper)
    else:
        in_band = (response <= settling_band_upper) & (response >= settling_band_lower)
    
    # Find first index where it enters band and stays
    settling_idx = None
    for i in range(len(in_band)):
        if in_band.iloc[i]:
            # Check if it stays in band for the rest of the signal
            if in_band.iloc[i:].sum() == len(in_band.iloc[i:]):
                settling_idx = i
                break
    
    if settling_idx is not None:
        settling_time = time.iloc[settling_idx]
        metrics['settling_time'] = settling_time
    else:
        metrics['settling_time'] = None
    
    metrics['settling_band_upper'] = settling_band_upper
    metrics['settling_band_lower'] = settling_band_lower
    
    return metrics

# Calculate metrics: Column 4 response to Column 3 setpoint
metrics = calculate_transient_metrics(time_relative, col_3, col_4, settling_tolerance=0.02)

# Step 4: Create the plot with annotations
fig, ax = plt.subplots(figsize=(16, 8))

# Plot setpoint (reference)
ax.plot(time_relative, col_3, 'g--', linewidth=2.5, label='Setpoint (Column 3)', alpha=0.8)

# Plot response
ax.plot(time_relative, col_4, 'b-', linewidth=2.5, label='Response (Column 4)')

# Target/Steady-state line
ax.axhline(y=metrics['target_value'], color='g', linestyle=':', linewidth=2, alpha=0.6, label='Target Value')

# Steady-state response line
ax.axhline(y=metrics['steady_state'], color='b', linestyle=':', linewidth=2, alpha=0.6, label='Steady-State Response')

# Peak marker
ax.plot(metrics['peak_time'], metrics['peak_value'], 'ro', markersize=12, label='Peak', zorder=5)
ax.annotate(f"Peak: {metrics['peak_value']:.4f}\nat {metrics['peak_time']:.4f}s\nOvershoot: {metrics['overshoot_pct']:.2f}%", 
            xy=(metrics['peak_time'], metrics['peak_value']),
            xytext=(0, -80), textcoords='offset points',
            bbox=dict(boxstyle='round,pad=0.7', fc='red', alpha=0.8, edgecolor='black', linewidth=1.5),
            arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0', color='red', lw=2),
            fontsize=10, color='white', fontweight='bold')

# Settling band
ax.fill_between(time_relative, metrics['settling_band_lower'], metrics['settling_band_upper'], 
                 alpha=0.15, color='yellow', label='±2% Settling Band')

# Settling time marker (if found)
if metrics['settling_time'] is not None:
    ax.axvline(x=metrics['settling_time'], color='orange', linestyle=':', linewidth=2.5, label='Settling Time')
    ax.annotate(f"Settling Time\n{metrics['settling_time']:.4f}s", 
                xy=(metrics['settling_time'], metrics['settling_band_upper']),
                xytext=(10, -30), textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.7', fc='orange', alpha=0.8, edgecolor='black', linewidth=1.5),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3', color='orange', lw=2),
                fontsize=10, fontweight='bold')

# Rise time markers (if found)
if metrics['rise_time'] is not None and metrics['rise_time'] > 0:
    ax.axvline(x=metrics['rise_time_10'], color='purple', linestyle='-.', linewidth=1.5, alpha=0.7)
    ax.axvline(x=metrics['rise_time_90'], color='purple', linestyle='-.', linewidth=1.5, alpha=0.7, label='10%-90% Points')
    
    # Annotate rise time
    mid_rise_time = (metrics['rise_time_10'] + metrics['rise_time_90']) / 2
    mid_rise_value = col_4.iloc[int(np.argmin(np.abs(time_relative - mid_rise_time)))]
    ax.annotate(f"Rise Time\n{metrics['rise_time']:.4f}s", 
                xy=(mid_rise_time, mid_rise_value),
                xytext=(-40, -30), textcoords='offset points',
                bbox=dict(boxstyle='round,pad=0.7', fc='purple', alpha=0.7, edgecolor='black', linewidth=1.5),
                arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3', color='purple', lw=2),
                fontsize=10, color='white', fontweight='bold')

# Create summary text box with all metrics
rise_time_str = f"{metrics['rise_time']:.4f} s" if metrics['rise_time'] is not None else 'N/A'
settling_time_str = f"{metrics['settling_time']:.4f} s" if metrics['settling_time'] is not None else 'Not settled'

summary_text = f"""TRANSIENT RESPONSE SUMMARY
━━━━━━━━━━━━━━━━━━━━━━━━━━
Target/Setpoint:           {metrics['target_value']:.4f}
Steady-State Response:     {metrics['steady_state']:.4f}
Steady-State Error:        {metrics['ss_error']:.4f}

Peak Value:                {metrics['peak_value']:.4f}
Peak Time:                 {metrics['peak_time']:.4f} s
Overshoot:                 {metrics['overshoot_pct']:.2f}%

Rise Time (10%-90%):       {rise_time_str}
Settling Time (±2%):       {settling_time_str}
"""

ax.text(0.02, 0.98, summary_text, transform=ax.transAxes, 
        fontsize=10, verticalalignment='top', family='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.9, edgecolor='black', linewidth=2),
        fontweight='bold')

ax.set_xlabel('Time (seconds)', fontsize=12, fontweight='bold')
ax.set_ylabel('Revolution (rev)', fontsize=12, fontweight='bold')
ax.set_title('Transient Response: System Response to Setpoint Change', fontsize=14, fontweight='bold')
ax.legend(loc='lower right', fontsize=10, framealpha=0.95)
ax.grid(True, alpha=0.3)
plt.tight_layout()

# Step 5: Display or save the plot
plt.show()
# plt.savefig('transient_analysis.png', dpi=300)  # Uncomment to save

# Step 6: Print detailed metrics
def print_metrics(metrics):
    print(f"\n{'='*60}")
    print(f"TRANSIENT RESPONSE ANALYSIS - COLUMN 4 vs COLUMN 3 (SETPOINT)")
    print(f"{'='*60}")
    print(f"Initial Response:         {metrics['initial_response']:.4f}")
    print(f"Target/Setpoint:          {metrics['target_value']:.4f}")
    print(f"Steady-State Response:    {metrics['steady_state']:.4f}")
    print(f"Steady-State Error:       {metrics['ss_error']:.4f}")
    print(f"\nPeak Value:               {metrics['peak_value']:.4f}")
    print(f"Peak Time:                {metrics['peak_time']:.4f} s")
    print(f"Overshoot:                {metrics['overshoot_pct']:.2f}%")
    
    if metrics['rise_time'] is not None:
        print(f"\nRise Time (10%-90%):      {metrics['rise_time']:.4f} s")
    else:
        print(f"\nRise Time (10%-90%):      Not found")
    
    if metrics['settling_time'] is not None:
        print(f"Settling Time (±2%):      {metrics['settling_time']:.4f} s")
    else:
        print(f"Settling Time (±2%):      Not found (signal doesn't settle)")
    
    print(f"{'='*60}")

print_metrics(metrics)