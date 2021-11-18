#include <IO.h>
#include <Configuration.h>
#include "BFS.h"

int main(int argc, char* argv[]) {
    Configuration start;
    Configuration target;

    // Read starting configuration
    std::ifstream inputStart;
    inputStart.open("start.in");
    IO::readConfiguration(inputStart, start);

    // Read target configuration
    std::ifstream inputTarget;
    inputTarget.open("target.in");
    IO::readConfiguration(inputTarget, target);

    // Get configs from start to target
    std::vector<Configuration> result = BFS(start, target, 90, 1);

    // Write the result of BFS search
    std::ofstream output;
    output.open("result.in");
    output << IO::toString(result);
    
}
