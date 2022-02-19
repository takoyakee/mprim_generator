# mprim_generator
This repository generates motion primitives that are compatible with [SBPL Library](http://wiki.ros.org/sbpl) for lattice state motion planning. It has been tested with this [planner](https://github.com/vvanirudh/sbpl_dynamic_adaptive_planner). 

## Dependency 
* ACADO. Install ACADO [here](https://acado.github.io/install_linux.html) and Instructions on building ACADO based executables are [here](https://sourceforge.net/p/acado/wiki/Using%20CMake%20-%20UNIX%20-%20Common) 

## Instructions

After cloning repo, run the following commands

` cd ~/(mprim_generator)/ `

` mkdir build `

` cd build && cmake .. `

` cd .. && ./mprim `

run ./csv_mprim_conveter.py: Input: csv filename, Output: mprim filename 


The generated .mprim files models vehicle constraints and can be used for lattice-state motion planning with the SBPL library
