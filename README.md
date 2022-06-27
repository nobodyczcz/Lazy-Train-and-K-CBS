## About
This repo contains codes for *Multi-Train Path Finding Revisited 
(Zhe Chen, Jiaoyang Li, Daniel Harabor, Peter Stuckey and Sven Koenig. SoCS 2022)* 
and *Symmetry Breaking for K-Robust Multi-Agent Path Finding (Zhe Chen, Daniel D Harabor, Jiaoyang Li, and Peter J Stuckey)*.

## Dependencies
1. boost
2. sparsehash
3. boost-python
4. Python-dev
5. Python and flatland-rl 2.2.2 if using flatland environment

Please make sure your boost-python version match your python version and boost version.

## Compilingßgit 
Use cmake to compile the program. You might need to adjust the cmake to suit your machine.

For compiling CBS-K, compile the codes under CBSH-rect-cmake folder.

```shell
mkdir build
cd build
cmake ../CBSH-rect-cmake
make -j
```

For compiling as a python lib, compile codes from PythonCBS.
```shell
mkdir python-lib-build
cd python-lib-build
cmake ../PythonCBS
make -j
```
## Usage
### CLI

#### Example
example instances provided under example folder.
```
CBS-K -m ./example/random-32-32-10.map -a ./example/random-32-32-10-scen-even/room-32-32-4-even-1.scen -o ./test -s CBSH-RM -t 120 --kDelay 4 --diff-k --screen 0 --corridor True --target True --shrink
```

#### See arguments and options:
```
CBS-K --help
```

#### Options:
**--shrink**    Run MTPF variant 2.

**--shrink --ignore-target**    Run MTPF variant 1.

Run MTPF variant 3 without above options.

**--ignore-train**  ignore head occupation or self conflict. 

Run K-robust CBS using **--ignore-train** 

**--kDelay 2**      Body length for trains/ or k value for k-cbs

**--diff-k**   generate body length uniformly and it use --kDelay as max k.

**--corridor True**     enable corridor reasoning

**--target True**   enable target reasoning

**--parking True**  enable parking reasoning


### Python Flatland
Place the compiled library libPythonCBS.xx together with python script.

See run-flatland experiment scripts for python script example.

## Experiment Scripts

The scripts and data for generating plots/tables in Multi-Train Path Finding Revisited
are under **exp-scripts** folder.

Scripts with "run-*.py" are used to run experiments
for different variants. 

Please specify the correct compiled executable and output folder
in the script before using them. The compiled python lib should be
placed in the same folder with "run-flatland-variant2.py".

The jupyter notebook file "plots.ipynb" including codes 
for generating plots and tables.
