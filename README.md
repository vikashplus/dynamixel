
# Python wrapper for dynamixels
Primarily designed for MX series

# Set up
0. Clone the [dynamixel robotis repo](https://github.com/ROBOTIS-GIT/DynamixelSDK.git). 
Note that the repo has gone through changes that aren't backward compatible. Checkout an earlier commit compatibility .
```
git clone https://github.com/ROBOTIS-GIT/DynamixelSDK.git  
git checkout 2ae37fc2390eedcd8fe356f40ece398720db0532 
```

1. build the c library

```
cd ~/Libraries/DynamixelSDK/c/build/linux64/ 
make 
```

2. add path to the ctype wrapper in the bashrc 

```export PYTHONPATH="/home/vik/Libraries/DynamixelSDK/python/dynamixel_functions_py:$PYTHONPATH"``` 

3. Edit `dynamixel_functions.py` to point to the right libraries (absolute path)


```dxl_lib = cdll.LoadLibrary("/home/vik/Libraries/DynamixelSDK/c/build/linux64/libdxl_x64_c.so")```

# usage
1. Open dynamixel_utils.py and pick the connected dynamixel type 
2. `python dynamixel_utils.py`to test
