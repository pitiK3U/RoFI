#include "BFS.h"
#include "dimcli/cli.h"

int main(int argc, char* argv[]) 
{
    Dim::Cli cli;
    auto & startPath = cli.opt<std::string>("start", "./start.in").desc("Starting configuration in valid format");
    auto & targetPath = cli.opt<std::string>("target", "./target.in").desc("Target configuration in valid format");
    auto & step = cli.opt<int>("step", 90).desc("Degree of rotation for 1 step");
    auto & bound = cli.opt<int>("bound", 1).desc("Bound");
    if (!cli.parse(argc, argv))
        return cli.printError(std::cerr); // prints error and returns cli.exitCode()
    
    Configuration start;
    Configuration target;

    // Read start configuration
    std::ifstream inputStart;
    inputStart.open(*startPath);
    if (inputStart.fail()) 
    {
        std::cerr << "Invalid path to start configuration: " + *startPath;
        exit(1);
    }
    if (!IO::readConfiguration(inputStart, start)) 
    {
        std::cerr << "Start configuration is not in valid format";
        exit(1);
    }
    if (!start.isValid()) 
    {
        std::cerr << "Start configuration is not valid";
        exit(1);
    }

    // Read target configuration
    std::ifstream inputTarget;
    inputTarget.open(*targetPath);
    if (inputTarget.fail()) 
    {
        std::cerr << "Invalid path to target configuration" + *targetPath;
        exit(1);
    }
    if (!IO::readConfiguration(inputTarget, target)) 
    {
        std::cerr << "Target configuration is not in valid format";
        exit(1);
    }
    if (!target.isValid()) 
    {
        std::cerr << "Start configuration is not valid";
        exit(1);
    }

    // Get configs from start to target
    std::vector<Configuration> result = BFS(start, target, *step, *bound);

    // Write the result of BFS search
    std::cout << IO::toString(result);
    
}
