mkdir -p build

cd build

cl ../src/Main.cpp ../src/API_Simulation.cpp ../src/Flood.cpp ../src/functions.cpp  /EHsc /DEBUG /Z7 /std:c++20