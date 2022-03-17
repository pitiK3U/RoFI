#include "BFS.h"
#include "dimcli/cli.h"

std::vector<Configuration> getDescendants(const Configuration& current, unsigned int step, unsigned int bound) 
{
    // Get possible configurations made from the current one
    // "1 step" away, ignoring configurations of identical classes
    // TODO

    std::vector<Configuration> possConfs;
    next(current, possConfs, step, bound);
    return possConfs;
}

int main(int argc, char* argv[]) 
{
    Dim::Cli cli;
    auto & startPath = cli.opt<std::string>("start", "./start.in").desc("Starting configuration in valid format");
    auto & targetPath = cli.opt<std::string>("target", "./target.in").desc("Target configuration in valid format");
    auto & step = cli.opt<int>("step", 90).desc("Degree of rotation for 1 step");
    auto & bound = cli.opt<int>("bound", 1).desc("Bound");
    auto & showStats = cli.opt<bool>("stats").desc("Show statistics of the BFS search");
    if (!cli.parse(argc, argv))
        return cli.printError(std::cerr); // prints error and returns cli.exitCode()
    
    Configuration start;
    Configuration target;
    BFSReporter reporter;

    // Read start configuration
    std::ifstream inputStart;
    inputStart.open(*startPath);
    if (inputStart.fail()) 
    {
        std::cerr << "Invalid path to start configuration: " + *startPath "\n";
        exit(1);
    }
    if (!IO::readConfiguration(inputStart, start)) 
    {
        std::cerr << "Start configuration is not in valid format" << "\n";
        exit(1);
    }
    if (!start.isValid()) 
    {
        std::cerr << "Start configuration is not valid" << "\n";
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
    std::vector<Configuration> result = BFS(start, target, *step, *bound, reporter);

    // Write the result of BFS search
    std::cout << IO::toString(result);

    // Show BFS stats
    if (*showStats) {
        std::cout << reporter.toString();
    }

    // Debugging
    // Prints all possible configurations of given modules

    /* Configuration cnf;
    std::ifstream inputTarget;
    inputTarget.open( "./basic.in" );
    IO::readConfiguration( inputTarget, cnf );
    std::cout << IO::toString( cnf ) << "\n";

    std::queue< const Configuration* > bfsQueue;
    std::unordered_set< Configuration, ConfigurationHash > seen;

    bfsQueue.push( &cnf );
    seen.insert( cnf );

    while ( !bfsQueue.empty() ) 
    {
        const Configuration* current = bfsQueue.front();

        bfsQueue.pop();

        std::vector<Configuration> descendants = getDescendants( *current, 90, 1 );

        for(Configuration child : descendants) 
        {
            if ( seen.find(child) == seen.end() ) 
            {
                auto iter = std::get<0>(seen.insert(child));

                // Points to configuration saved in seen, not to the temporary one here
                const Configuration* child_ptr = &(*iter);

                bfsQueue.push(child_ptr);
            }
        }
    }

    int i = 0;

    for ( const Configuration &c : seen ) {
        std::string fileName = "./configs/" + std::to_string( i ) + ".in";
        std::cout << fileName << "\n";
        std::ofstream myFile( fileName );
        myFile << IO::toString( c ) << "\n";
        i++;
    }

    std::cout << seen.size() << "\n"; */
}
