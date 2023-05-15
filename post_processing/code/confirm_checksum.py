from pathlib import Path
from sys import argv
from crc import Calculator, Crc8

crc_algo = Calculator(Crc8.AUTOSAR)

path = Path(__file__).parent.parent / "data" / "crc_check.txt"

lines = path.read_text().splitlines()

for idx, line in enumerate(lines):
    print(
        f"Line {idx} CRC {'PASS' if crc_algo.verify(bytes(line[:-1], encoding='utf-8'), bytes(line[-1], encoding='utf-8')) else 'FAIL'}"
    )
