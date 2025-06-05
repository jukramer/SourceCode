mkdir -p build

cd build

cl /O2 /GL /DNDEBUG /EHsc ../src/Main.cpp ../src/API_Simulation.cpp ../src/functions.cpp ../src/Flood.cpp /link /LTCG /std:c++20
