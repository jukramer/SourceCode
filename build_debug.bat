mkdir -p build

cd build

cl ../src/Main.cpp ../src/API_Simulation.cpp ../src/functions.cpp ../src/Flood.cpp  /EHsc /DEBUG /Z7 /std:c++20