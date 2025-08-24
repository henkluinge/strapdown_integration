# Strapdown integration
C and python reference code for MEMS IMU type of strapdown integration: compute orientation and velocity change by inertial sensor integration.

From math to imu:
- A [strapdown report](https://github.com/henkluinge/strapdown_integration/blob/main/tex/infertia_strapdown_integration.pdf), containing the math and definition of intervals.
- Python reference code `reference_strapdown_integration.ipynb`. It generates a reference log for comparison against a C implementation.
- C implementation.


### Dependencies
Python: `numpy`, `pandas`

### Compile and run
Generating a test-dataset with c-code is easiest with make:

``` 
mkdir build
cd build
cmake ..
make
./strapdown ../tests/simple_strapdown_reference.csv
```