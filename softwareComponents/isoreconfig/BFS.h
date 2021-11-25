#include <vector>
#include <set>
#include <Configuration.h>
#include <map>
#include <Generators.h>
#include <IO.h>
#include <queue>


std::vector<Configuration> BFS(const Configuration& start, const Configuration& target, 
unsigned int step, unsigned int bound, BFSReporter reporter);
