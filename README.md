# Strapdown integration
Reference code for MEMS IMU type of strapdown integration: compute orientation and velocity change by inertial sensor integration.

From math to imu:
- A [strapdown report](https://github.com/henkluinge/strapdown_integration/blob/main/tex/infertia_strapdown_integration.pdf), containing the math and definition of intervals.
- Python reference code `reference_strapdown_integration.ipynb`. It generates a reference log for comparison agains a C implementation.
- C implementation as a starting point to ported to a uC target.


