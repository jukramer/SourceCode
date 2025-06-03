mkdir -p build

cd build

cl ../src/Main.cpp ../src/API_Simulation.cpp /EHsc /DEBUG /Z7 /std:c++20