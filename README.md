## Dependencies
1. boost
2. boost-python
3. Python-dev
4. Python and flatland-rl 2.2.2 if using flatland environment

Please make sure your boost-python version match the python version.

## Compiling
Use cmake to compile the program. You might need to adjust the cmake to suit your machine.

For compiling CBS-K, compile the codes under CBSH-rect-cmake folder.

For compiling as a python lib, compile codes from PythonCBS.

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
**--shrink**    MTPF variant 2, without this option will run in MTPF variant 3

**--shrink --ignore-target**    MTPF variant 1 with these two options

**--kDelay 2**      Body length for trains

**--diff-k**   generate body length uniformly and use --kDelay as max k.

**--corridor True**     enable corridor reasoning

**--target True**   enable target reasoning

**--parking True**  enable parking reasoning

**--ignore-train**  ingore head occupation or self conflict. (run in k-robust mode)

### Python Flatland
Place the compiled library libPythonCBS.xx together with python script.

See test.py under PythonCBS for python script example.
