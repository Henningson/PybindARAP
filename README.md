# PybindARAP
Implementation of the As-Rigid-As-Possible Deformation in C++ with python wrappings.

# Dependencies
The only other library you need for this ARAP implementation to work is ![Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page). 

# Build
First, make sure that Eigen is installed. If not, you can install it with:
```
sudo apt install libeigen3-dev
```
Then, clone this repository including the pybind11 submodule
```
git clone --recurse-submodules https://github.com/Henningson/PybindARAP.git .
```
After that, build it.
```
cd PybindARAP
mkdir build
cd build
cmake ..
make
cd ..
```
An example is given in example.py. You can test the code with:
```
python example.py
```

# Acknowledgements
A big thank you to **Daniel Zint** for supplying us with the base code, that transformed into this little code-snippet.
