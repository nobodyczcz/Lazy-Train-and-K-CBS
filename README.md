## Dependencies
1. boost
2. boost-python

boost-python is for building a python-c++ api, in order to run experiments on flatland railway maps.
You can remove it and all other places that uses things from python. The heading stuff in low lovel search is also for 
flatland map.

## Compiling
Use cmake to compile the program. You might need to adjust the cmake to suit your machine.

For compiling CBS-K, compile the codes under CBSH-rect-cmake folder.

For compiling as a python lib, compile codes from PythonCBS.

## Usage

```
CBS-K -m ./example/random-32-32-10.map -a ./example/32_32_20_12_k2_rm3.agents -o ./test -s CBSH-RM -t 120 --kDelay 2 --screen 0 --corridor2 false --target false
```