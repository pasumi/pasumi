# pasumi

A simulator for developing and testing redirected walking controllers.

## Installation

* Clone the repository in your location of choice: ```git clone https://github.com/pasumi/pasumi.git```

- On **Windows:** 

    - Open CMake and navigate to the location where you cloned the pasumi library.

    - Click _Configure_ and then _Generate_. This will create a file called ```nori.sln```, which you can double-click to open the project in Visual Studio.

    - Press _Build -> Build Solution_ to compile the library, after which you can run experiments or use the code as you please.

* On **Linux:** 

    - In terminal, navigate to the top level of the cloned pasumi library.

    - Build ```pasumi``` by running the following command from the terminal: ```cmake .```

    - Compile the library using the following command: ```make```

    - Run the compiled code using the command: ```./pasumi```