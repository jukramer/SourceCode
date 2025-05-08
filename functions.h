#include <vector>
#include <algorithm>
#include <string>

#ifndef FUNCTIONS_H
#define FUNCTIONS_H

int manhattanDist(std::vector<int>, std::vector<std::vector<int>>);
std::vector<std::string> stateMachineSimple(const std::string &path);
std::vector<std::string> stateMachine(const std::string &path);

#endif
