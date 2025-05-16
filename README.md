# Robust-Plans-KR24
Source code and benchmark data for the paper "On Generating Robust Plans and Linear Execution Strategies in Planning Against Nature" published at ICAPS 2022

# Compiling the code

- Set up a path to fastdownward in `include/paths.h` (fastdownward can be obtained at fast-downward.org)

type `make`

# Running the benchmarks

run `benchmarks/run-<gen/les>-bench.sh` (it should be run from the main directory, where the compiled binaries are)

results can be found at `benchmarks/results/`



# References to Existing Code (on which our code is built)

The compiler to FOND (benchmarks/toFOND/) has been taken from: https://github.com/martinpilat/events-FOND 
The PDDL translator (modified FastDownward one) and the ``SAS representation'' classes have been taken from: https://gitlab.com/ctu-fee-fras/public/server-client-simulator-kr-2021 
Robust Plan Verification and Generation: https://github.com/lchrpa/Robust-Plans-KR24
Linear Execution Strategy Verification: https://github.com/lchrpa/Verification-of-Plans-Against-Nature
