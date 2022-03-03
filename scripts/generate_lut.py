__doc__ = """
    To run:
        cd /path/to/this/file
        python3 generate_lut.py

    Generates a lookup table in your current working directory.
    Edit the LUT_NAME, LUT_FUNC, and LUT_SIZE variables    to generate the LUT that you need.

    Note: the size of the LUT in bytes will be 4*LUT_SIZE
"""

import numpy as np

COL_SIZE = 8

LUT_NAME = "ADC_TO_TEMP_LUT"
LUT_SIZE = (1 << 12) - 1

# arguments: x is the adc value ranging from 0 to 4096
def LUT_FUNC(x):
    # resistor value at 25 C
    r_nominal = 10000
    # resistor value used in voltage divider
    r1 = 10000
    B = 3435
    # Room temperature in Kelvin
    room_t_kelvin = 298.15

    v_out =  x * (3.3/4096)

    # round here to avoid division by zero error 
    if v_out == 3.3:
        v_out = 3.2999999

    # calculate thermistor resistance value using voltage divider equation
    r_thermistor = (-10000 * v_out) / (v_out - 3.3)
    
    # Apply Steinhart-Hart equation and convert to celcius
    T_kelvin = 1 / ( (1/298.15) + (1/B)*np.log( r_thermistor / r_nominal )  )
    T_celcius = T_kelvin - 273.15
    
    return T_celcius

def main():
    lut = [LUT_FUNC(x) for x in range(LUT_SIZE)]

    with open(f"{LUT_NAME}.c", "w") as f:
        f.write(f"const float {LUT_NAME}[{LUT_SIZE}] = {{\n\t")

        for i, lut_i in enumerate(lut):
            f.write(f"{lut_i:.7f}")    # floats in C have max 7 decimal places of precision

            if i != (len(lut) - 1):
                f.write(", ")

            if ((COL_SIZE - 1) - i) % COL_SIZE == 0:
                f.write("\n")
                if (i != (len(lut) - 1)):
                    f.write(" "*4)

        f.write("};")

if __name__ == "__main__":
    main()
