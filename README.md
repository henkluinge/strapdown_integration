# Strapdown integration
C and python reference code for MEMS IMU type of strapdown integration: compute orientation and velocity change by inertial sensor integration.

From math to imu:
- A [strapdown report](https://github.com/henkluinge/strapdown_integration/blob/main/tex/infertia_strapdown_integration.pdf), containing the math and definition of intervals.
- Python reference code `reference_strapdown_integration.ipynb`. It generates a reference log for comparison against a C implementation.
- C implementation as a starting point for a port to a uC target.


### Dependencies
Python: 
- numpy, pandas

C: 
- Quaternion lib: https://github.com/MartinWeigel/Quaternion