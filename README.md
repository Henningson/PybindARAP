# PybindARAP
Implementation of the As-Rigid-As-Possible Deformation in C++ with python wrappings.

## Dependencies
The only other library you need for this ARAP implementation to work is ![Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). 

## Build
First, make sure that Eigen is installed. If not, you can install it (on Linux) with:
```
sudo apt install libeigen3-dev
```
Windows users can get eigen <a href="https://eigen.tuxfamily.org/index.php?title=Main_Page">here</a>.
You need to create a new environment variable `EIGEN3_INCLUDE_DIR`, and specify the root of eigen, i.e. `EIGEN3_INCLUDE_DIR=C:/eigen-3.4.0`.
If you want to set the environment variable inside a conda environment, use:
```
conda env config vars set EIGEN3_INCLUDE_DIR=%EIGEN_ROOT_DIR%
```
Where, %EIGEN_ROOT_DIR% is the root directory of Eigen.  
Then, clone this repository including the pybind11 submodule
```
git clone --recurse-submodules https://github.com/Henningson/PybindARAP.git .
```
and install it using
```
python3 -m pip install ./PybindARAP/

or

cd PybindARAP
python setup.py install
```
Or you may build it locally using
```
cd PybindARAP
mkdir build
cd build
cmake ..
make
```
An example is given in example.py. You can test the code with:
```
python example.py
```

## Acknowledgements
A big thank you to **Daniel Zint** for supplying us with the base code that transformed into this little code-snippet.
If you are interested in some awesome Geometry Processing work, check out ![his GitHub Account](https://github.com/DanielZint).
