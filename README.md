# PybindARAP
Implementation of the As-Rigid-As-Possible Deformation in C++ with python wrappings.

## Dependencies
The only other library you need for this ARAP implementation to work is ![Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). 

## Build
First, make sure that Eigen is installed. If not, you can install it (on Linux) with:
```
sudo apt install libeigen3-dev
```
Then, clone this repository including the pybind11 submodule
```
git clone --recurse-submodules https://github.com/Henningson/PybindARAP.git .
```
and install it using
```
python3 -m pip install ./PybindARAP/
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
