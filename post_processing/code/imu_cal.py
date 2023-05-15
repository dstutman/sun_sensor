from pathlib import Path
import numpy as np
import pandas as pd
import plotly.express as px

data_root = Path(__file__).parent.parent / "data"

static_data = pd.read_csv(
    data_root / "static.csv",
    names=[
        "adc_0",
        "adc_1",
        "adc_2",
        "adc_3",
        "adc_4",
        "adc_5",
        "adc_6",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "mag_x",
        "mag_y",
        "mag_z",
    ],
    sep=",",
    header=None,
    skiprows=[0, -1],
)

dynamic_data = pd.read_csv(
    data_root / "dynamic.csv",
    names=[
        "adc_0",
        "adc_1",
        "adc_2",
        "adc_3",
        "adc_4",
        "adc_5",
        "adc_6",
        "acc_x",
        "acc_y",
        "acc_z",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "mag_x",
        "mag_y",
        "mag_z",
    ],
    sep=",",
    header=None,
    skiprows=[0, -1],
)

## Static calibration
acc = static_data[["acc_x", "acc_y", "acc_z"]]
gyro = static_data[["gyro_x", "gyro_y", "gyro_z"]]

# Compute accelerometer calibration parameters
print(f"Accelerometer bias (g) : {acc.mean().to_numpy() - np.array([0, 0, 1])}")
print(f"Accelerometer variance (g2) : {acc.var().to_numpy()}")

# Compute gyroscope calibration parameters
print(f"Gyro bias (rad s-1) : {gyro.mean().to_numpy()}")
print(f"Gyro variance (rad2 s-2) : {gyro.var().to_numpy()}")

## Dynamic calibration
mag = dynamic_data[["mag_x", "mag_y", "mag_z"]]

# Compute magnetometer calibration parameters
# TODO: This is probably wrong due to uneven distribution of sample orientations
soft_iron_error = mag.mean().to_numpy()
recentered_field = mag - soft_iron_error

total_field = recentered_field.apply(np.linalg.norm, axis=1)

print(f"Soft iron error (gauss) : {soft_iron_error}")
print(f"Mean local field strength (gauss): {total_field.mean()}")
print(f"Mag non-sphericity (gauss2) : {total_field.var()}")

mag_fig = px.scatter_3d(recentered_field, x="mag_x", y="mag_y", z="mag_z")
mag_fig.show()
