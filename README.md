# pasumi

A simulator for developing and testing redirected walking controllers.

## Installation

* Clone the repository in your location of choice: ```git clone https://github.com/pasumi/pasumi.git```

- On **Windows:** 

    - Open CMake and navigate to the location where you cloned the pasumi library.

    - Click _Configure_ and then _Generate_. This will create a file called ```pasumi.sln```, which you can double-click to open the project in Visual Studio.

    - Press _Build -> Build Solution_ to compile the library, after which you can run experiments or use the code as you please.

* On **Linux:** 

    - In terminal, navigate to the top level of the cloned pasumi library.

    - Run the following commands in the terminal:
    ```
    mkdir build
    cd build
    cmake ..
    cmake --build .
    ```

If you would like to use the visibility polygon-based RDW controller (described in [this](https://gamma.umd.edu/researchdirections/vrlocomotion/vis_poly/) paper), you will need to install [boost](https://www.boost.org/) and [this visibility polygon library (and its dependencies)](https://github.com/idlebear/VisibilityPolygon).
    
## Quick-start Guide

- Download and compile the library (see above).

- Create a new class that inherits from the `redirector` class. This will be your redirection controller.

- Implement the constructor and `update()` methods of your newly-created class. See `arc.cpp` for an example.

- Edit `config.h` to set the simulation parameters as you wish.

- Compile and run!