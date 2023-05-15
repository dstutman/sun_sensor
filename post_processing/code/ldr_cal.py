from pathlib import Path
import pandas as pd
from math import log2

data_root = Path(__file__).parent.parent / "data"

ldr_data = pd.read_csv(
    data_root / "ldr.csv",
    names=[
        "adc_0",
        "adc_1",
        "adc_2",
        "adc_3",
        "adc_4",
        "adc_5",
    ],
    sep=",",
    header=None,
    skiprows=[0, -1],
)


def resistance_from_adc(sample):
    return (1 / sample - 1) * 1e5  # LDR is paired with a 100kR


print(f"Maximum resistance: {ldr_data.min().apply(resistance_from_adc)}\n")

print(f"Minimum resistance: {ldr_data.max().apply(resistance_from_adc)}\n")

print(
    f"Usable bits of resolution: {(ldr_data.max() - ldr_data.min()).mul(2**12).apply(log2).apply(int)}\n"
)
