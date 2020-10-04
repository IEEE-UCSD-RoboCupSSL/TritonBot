# TritonBot



### build

build project by cmake, check out CMakeLists.txt

To build
```
mkdir build
cd build
cmake .. 
make
```
Upon modifications were made in CMakeLists.txt, rerun cmake by:
```
make clean
cmake ..
make
```

### Project Structure

........ run tree


### Google Test
```
git submodule init
git submodule update
```

### Developer Note
- Remember to update CMakeList after adding more srcs or src dirs. Failing to do so has resulted in the notorious "vtable" error multiple times.
