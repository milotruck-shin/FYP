import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ============================================
# Transient Response Analysis - CSV Data
# ============================================

# Step 1: Read the CSV file
csv_file = r"D:\accel 20\csv\kp 2 kd 0_05.csv"


def extract_numeric(value):
    """Extract numeric value from strings with units."""
    if isinstance(value, str):
        import re
        match = re.search(r'-?\d+\.?\d*', value.strip())
        if match:
            return float(match.group())
    return float(value)


try:
    # Read CSV without assuming there's a header row
    df = pd.read_csv(csv_file, header=None)

    print(
        f"Successfully loaded CSV file with "
        f"{len(df)} rows and {len(df.columns)} columns"
    )

    print(f"First row: {df.iloc[0].tolist()}")
    print("kp 2_5 kd 0_05.csv")

except FileNotFoundError:
    print(f"Error: File '{csv_file}' not found.")
    exit()


# ============================================
# Step 2: Extract data
# ============================================

# Column 0 = Time
# Column 2 = Setpoint
# Column 3 = Response

time_raw = df.iloc[:, 0].apply(extract_numeric)
col_3_raw = df.iloc[:, 2].apply(extract_numeric)
col_4_raw = df.iloc[:, 3].apply(extract_numeric)
torque_col = df.iloc[:, 5].apply(extract_numeric)
current_col = df.iloc[:, 8].apply(extract_numeric)

max_value_torque = torque_col.max()
min_value_torque = torque_col.min()
max_value_current = current_col.max()
min_value_current = current_col.min()

print(f"Max value torque: {max_value_torque:.2f}, Min value torque: {min_value_torque:.2f} ")
print(f"Max current: {max_value_current:.2f}, Min current: {min_value_current:.2f} ")


# Convert milliseconds to seconds
time_s = time_raw / 1000

# Start time from zero
time_relative = time_s - time_s.iloc[0]

# Data
time_ms = time_relative * 1000
col_3 = col_3_raw
col_4 = col_4_raw


# ============================================
# Step 3: Calculate transient response metrics
# ============================================

def calculate_transient_metrics(
    time,
    setpoint,
    response,
    settling_tolerance=0.02
):
    """
    Calculate transient response metrics.

    setpoint: desired reference signal
    response: actual system response
    settling_tolerance: 0.02 = ±2%
    """

    metrics = {}

    # ----------------------------------------
    # Target / Setpoint
    # ----------------------------------------

    target_value = setpoint.iloc[-1]

    metrics['target_value'] = target_value

    # ----------------------------------------
    # Estimate steady-state response
    # ----------------------------------------
    # Instead of using only the final sample,
    # average the final 10% of the response.

    n_ss = max(1, int(len(response) * 0.10))

    ss_response = response.iloc[-n_ss:].mean()

    metrics['steady_state'] = ss_response

    # Steady-state error
    metrics['ss_error'] = abs(
        ss_response - target_value
    )

    # ----------------------------------------
    # Initial response
    # ----------------------------------------

    initial_response = response.iloc[0]

    metrics['initial_response'] = initial_response

    # ========================================
    # OVERSHOOT
    # ========================================

    # Determine whether this is a step-up
    # or step-down response.

    response_change = target_value - initial_response

    if response_change > 0:

        # ------------------------------------
        # Step UP
        # ------------------------------------

        # Maximum response value
        peak_idx = response.idxmax()
        peak_value = response.loc[peak_idx]

        # Overshoot above target
        overshoot_pct = max(
            0,
            (peak_value - target_value)
            / abs(response_change)
            * 100
        )

    elif response_change < 0:

        # ------------------------------------
        # Step DOWN
        # ------------------------------------

        # Minimum response value
        peak_idx = response.idxmin()
        peak_value = response.loc[peak_idx]

        # Overshoot below target
        overshoot_pct = max(
            0,
            (target_value - peak_value)
            / abs(response_change)
            * 100
        )

    else:

        peak_idx = response.idxmax()
        peak_value = response.loc[peak_idx]

        overshoot_pct = 0

    peak_time = time.loc[peak_idx]

    metrics['peak_value'] = peak_value
    metrics['peak_time'] = peak_time
    metrics['overshoot_pct'] = overshoot_pct

    # ========================================
    # RISE TIME
    # ========================================

    response_ss_change = target_value - initial_response

    if response_ss_change > 0:

        rise_10 = initial_response + 0.1 * response_ss_change
        rise_90 = initial_response + 0.9 * response_ss_change

        idx_10_cross = np.where(response >= rise_10)[0]
        idx_90_cross = np.where(response >= rise_90)[0]

    elif response_ss_change < 0:

        rise_10 = initial_response + 0.1 * response_ss_change
        rise_90 = initial_response + 0.9 * response_ss_change

        idx_10_cross = np.where(response <= rise_10)[0]
        idx_90_cross = np.where(response <= rise_90)[0]

    else:

        idx_10_cross = []
        idx_90_cross = []

    # ----------------------------------------
    # Interpolate crossing time
    # ----------------------------------------

    if len(idx_10_cross) > 0 and len(idx_90_cross) > 0:

        idx_10 = idx_10_cross[0]
        idx_90 = idx_90_cross[0]

        # ---- 10% crossing ----

        if idx_10 > 0:

            t1 = time.iloc[idx_10 - 1]
            t2 = time.iloc[idx_10]

            v1 = response.iloc[idx_10 - 1]
            v2 = response.iloc[idx_10]

            if v2 != v1:

                time_10 = (
                    t1
                    + (rise_10 - v1)
                    * (t2 - t1)
                    / (v2 - v1)
                )

            else:
                time_10 = t2

        else:

            time_10 = time.iloc[idx_10]

        # ---- 90% crossing ----

        if idx_90 > 0:

            t1 = time.iloc[idx_90 - 1]
            t2 = time.iloc[idx_90]

            v1 = response.iloc[idx_90 - 1]
            v2 = response.iloc[idx_90]

            if v2 != v1:

                time_90 = (
                    t1
                    + (rise_90 - v1)
                    * (t2 - t1)
                    / (v2 - v1)
                )

            else:
                time_90 = t2

        else:

            time_90 = time.iloc[idx_90]

        rise_time = abs(time_90 - time_10)

        metrics['rise_time'] = rise_time
        metrics['rise_time_10'] = time_10
        metrics['rise_time_90'] = time_90

    else:

        metrics['rise_time'] = None


    # ========================================
    # SETTLING TIME
    # ========================================

    settling_band_upper = (
        target_value
        + settling_tolerance * abs(target_value)
    )

    settling_band_lower = (
        target_value
        - settling_tolerance * abs(target_value)
    )

    # Determine whether every point is inside
    # the settling band.

    in_band = (
        (response >= settling_band_lower)
        &
        (response <= settling_band_upper)
    )

    # ----------------------------------------
    # Find FIRST point after which ALL
    # remaining points stay inside the band.
    #
    # This is stricter than the original
    # 80% criterion and properly accounts
    # for oscillations.
    # ----------------------------------------

    settling_idx = None

    for i in range(len(in_band)):

        if in_band.iloc[i]:

            if in_band.iloc[i:].all():

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


# ============================================
# Calculate metrics
# ============================================

metrics = calculate_transient_metrics(
    time_relative,
    col_3,
    col_4,
    settling_tolerance=0.02
)


# ============================================
# Step 4: Create plot
# ============================================

fig, ax = plt.subplots(figsize=(16, 8))

# Response
ax.plot(
    time_ms,
    col_4,
    'b-',
    linewidth=2.5,
    label='Current position (rev)'
)

# Setpoint
ax.axhline(
    y=metrics['target_value'],
    color='green',
    linestyle='--',
    linewidth=2,
    label='Setpoint'
)

# Estimated steady-state
ax.axhline(
    y=metrics['steady_state'],
    color='b',
    linestyle=':',
    linewidth=2,
    alpha=0.6,
    label='Steady-State Response'
)

# --------------------------------------------
# Peak marker
# --------------------------------------------

ax.plot(
    metrics['peak_time'] * 1000,
    metrics['peak_value'],
    'ro',
    markersize=12,
    label='Peak',
    zorder=5
)

ax.annotate(
    f"Peak: {metrics['peak_value']:.4f}\n"
    f"Time: {metrics['peak_time'] * 1000:.2f} ms\n"
    f"Overshoot: {metrics['overshoot_pct']:.2f}%",
    xy=(
        metrics['peak_time'] * 1000,
        metrics['peak_value']
    ),
    xytext=(0, -100),
    textcoords='offset points',
    bbox=dict(
        boxstyle='round,pad=0.6',
        fc='red',
        alpha=0.85,
        edgecolor='darkred',
        linewidth=2
    ),
    arrowprops=dict(
        arrowstyle='->',
        lw=2.5,
        color='darkred'
    ),
    fontsize=9,
    color='white',
    fontweight='bold',
    ha='center'
)

# --------------------------------------------
# Settling band
# --------------------------------------------

ax.axhline(
    y=metrics['settling_band_upper'],
    color='yellow',
    linestyle='--',
    linewidth=1.5,
    alpha=0.6
)

ax.axhline(
    y=metrics['settling_band_lower'],
    color='yellow',
    linestyle='--',
    linewidth=1.5,
    alpha=0.6,
    label='±2% Settling Band'
)

# --------------------------------------------
# Settling time
# --------------------------------------------

if metrics['settling_time'] is not None:

    ax.axvline(
        x=metrics['settling_time'] * 1000,
        color='orange',
        linestyle=':',
        linewidth=2.5,
        label='Settling Time'
    )

    y_middle = (
        ax.get_ylim()[0]
        + (
            ax.get_ylim()[1]
            - ax.get_ylim()[0]
        ) * 0.5
    )

    ax.text(
        metrics['settling_time'] * 1000,
        y_middle,
        f"Settling Time\n"
        f"{metrics['settling_time'] * 1000:.2f} ms",
        ha='center',
        va='center',
        bbox=dict(
            boxstyle='round,pad=0.8',
            fc='orange',
            alpha=0.85,
            edgecolor='black',
            linewidth=2
        ),
        fontsize=10,
        fontweight='bold'
    )


# --------------------------------------------
# Rise time
# --------------------------------------------

if metrics['rise_time'] is not None:

    ax.axvline(
        x=metrics['rise_time_10'] * 1000,
        color='purple',
        linestyle='-.',
        linewidth=2,
        alpha=0.8
    )

    ax.axvline(
        x=metrics['rise_time_90'] * 1000,
        color='purple',
        linestyle='-.',
        linewidth=2,
        alpha=0.8,
        label='10%-90% Points'
    )

    mid_rise_time = (
        metrics['rise_time_10']
        + metrics['rise_time_90']
    ) / 2 * 1000

    y_pos = (
        ax.get_ylim()[0]
        + (
            ax.get_ylim()[1]
            - ax.get_ylim()[0]
        ) * 0.05
    )

    ax.text(
        mid_rise_time,
        y_pos,
        f"Rise Time: "
        f"{metrics['rise_time'] * 1000:.4f} ms",
        ha='center',
        va='bottom',
        bbox=dict(
            boxstyle='round,pad=0.8',
            fc='purple',
            alpha=0.85,
            edgecolor='black',
            linewidth=2
        ),
        fontsize=10,
        color='white',
        fontweight='bold'
    )


# ============================================
# Plot formatting
# ============================================

ax.set_xlabel(
    'Time (milliseconds)',
    fontsize=12,
    fontweight='bold'
)

ax.set_ylabel(
    'Revolution (rev)',
    fontsize=12,
    fontweight='bold'
)

ax.set_title(
    'Transient Response: Kp 2.5 Kd 0.05',
    fontsize=14,
    fontweight='bold'
)

ax.legend(
    loc='lower right',
    fontsize=10,
    framealpha=0.95
)

ax.grid(
    True,
    alpha=0.3
)

plt.tight_layout()


# ============================================
# Step 5: Print metrics
# ============================================

def print_metrics(metrics):

    print("\n" + "=" * 80)

    print(
        "TRANSIENT RESPONSE ANALYSIS - "
        "Current Position vs Position (SETPOINT)"
    )

    print("=" * 80)

    print(
        f"Initial Response:         "
        f"{metrics['initial_response']:.6f}"
    )

    print(
        f"Target/Setpoint:          "
        f"{metrics['target_value']:.6f}"
    )

    print(
        f"Steady-State Response:    "
        f"{metrics['steady_state']:.6f}"
    )

    print(
        f"Steady-State Error:       "
        f"{metrics['ss_error']:.6f}"
    )

    print()

    print(
        f"Peak Value:               "
        f"{metrics['peak_value']:.6f}"
    )

    print(
        f"Peak Time:                "
        f"{metrics['peak_time'] * 1000:.4f} ms"
    )

    print(
        f"Overshoot:                "
        f"{metrics['overshoot_pct']:.2f}%"
    )

    # ----------------------------------------
    # Rise time
    # ----------------------------------------

    if metrics['rise_time'] is not None:

        print()

        print(
            f">>> Rise Time (10%-90%): "
            f"{metrics['rise_time'] * 1000:.4f} ms <<<"
        )

        print(
            f"    10% crossing at:      "
            f"{metrics['rise_time_10'] * 1000:.4f} ms"
        )

        print(
            f"    90% crossing at:      "
            f"{metrics['rise_time_90'] * 1000:.4f} ms"
        )

    else:

        print(
            "\nRise Time (10%-90%):      "
            "Not found"
        )

    # ----------------------------------------
    # Settling time
    # ----------------------------------------

    if metrics['settling_time'] is not None:

        print(
            f"\nSettling Time (±2%):      "
            f"{metrics['settling_time'] * 1000:.4f} ms"
        )

        print(
            f"  Settling Band:         "
            f"[{metrics['settling_band_lower']:.6f}, "
            f"{metrics['settling_band_upper']:.6f}]"
        )

    else:

        print(
            "\nSettling Time (±2%):      "
            "Not found (signal doesn't settle)"
        )

        print(
            f"  Settling Band:         "
            f"[{metrics['settling_band_lower']:.6f}, "
            f"{metrics['settling_band_upper']:.6f}]"
        )

    print("=" * 80)
    print()


# ============================================
# Print results
# ============================================

print_metrics(metrics)


# ============================================
# Step 6: Display plot
# ============================================

print("Showing plot... (Close the plot window to finish)")

plt.show()

# Uncomment to save:
# plt.savefig('transient_analysis.png', dpi=300)
