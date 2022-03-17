#include "BFS.h"
#include <cassert>

const Configuration getRepre(const Configuration& sample) 
{
    // Get a configuration and return a different one, 
    // which represents the class the input configuration belongs to
    // TODO

    return sample;
}

bool equalConfig(const Configuration& first, const Configuration& second) 
{

    // Compare if first and second are equal configurations
    // (they belong to the same isomorphic class)
    // TODO

    return first == second;
}

std::vector<Configuration> getDescendants(const Configuration& current, unsigned int step, unsigned int bound) 
{
    // Get possible configurations made from the current one
    // "1 step" away, ignoring configurations of identical classes
    // TODO

    std::vector<Configuration> possConfs;
    next(current, possConfs, step, bound);
    return possConfs;
}

std::vector<Configuration> getPredecessors(std::unordered_map<const Configuration*, const Configuration*>& predecessor, 
    const Configuration& target, const Configuration& start)
{
    // Make a vector of predecessors of target configuration 
    // from map of predecessors
    // Assume target is in predecessors

    std::vector<Configuration> output;
    const Configuration* current = &target;
    
    while (!equalConfig(*current, start)) {
        assert(current);
        output.push_back(*current);
        // current config must have a predecessor
        assert(predecessor.find(current) != predecessor.end());
        current = predecessor.find(current)->second;
    }

    output.push_back(start);

    std::reverse(output.begin(), output.end());
    return output;

} 

std::vector<Configuration> BFS(const Configuration& start, const Configuration& target, 
    unsigned int step, unsigned int bound, BFSReporter& reporter)
{
    const Configuration s = getRepre(start);
    const Configuration t = getRepre(target);

    std::unordered_map<const Configuration*, const Configuration*> predecessor;
    std::unordered_map<const Configuration*, int> distance;

    // Starting configuration has itself as predecessor
    // and distance of 0
    predecessor.insert( { &s, &s } );
    reporter.onUpdatePredecessors( predecessor );
    distance.insert( { &s, 0 } );
    reporter.onUpdateDistance( distance );

    // start is isomorphic to target
    if ( equalConfig( s, t ) ) 
        return getPredecessors( predecessor, s, s );

    std::queue< const Configuration* > bfsQueue;
    std::unordered_set< Configuration, ConfigurationHash > seen;

    bfsQueue.push( &s );
    reporter.onUpdateQueue( bfsQueue );

    seen.insert( s );
    reporter.onUpdateSeen( seen );

    while ( !bfsQueue.empty() ) 
    {
        const Configuration* current = bfsQueue.front();
        reporter.onUpdateCurrent( *current );

        bfsQueue.pop();
        reporter.onUpdateQueue( bfsQueue );

        std::vector<Configuration> descendants = getDescendants(*current, step, bound);

        for(Configuration child : descendants) {
            if (seen.find(child) == seen.end()) {
                auto iter = std::get<0>(seen.insert(child));
                reporter.onUpdateSeen(seen);

                // Points to configuration saved in seen, not to the temporary one here
                const Configuration* child_ptr = &(*iter);

                predecessor.insert({child_ptr, current});
                reporter.onUpdatePredecessors(predecessor);

                distance.insert({child_ptr, distance.find(current)->second + 1});
                reporter.onUpdateDistance(distance);

                if (equalConfig(*child_ptr, t)) 
                {
                    std::vector<Configuration> output = getPredecessors(predecessor, *child_ptr, s);
                    reporter.onBuildPredecessors(output);
                    return output;
                }

                bfsQueue.push(child_ptr);
                reporter.onUpdateQueue(bfsQueue);
            }
        }
    }
    std::vector<Configuration> output = {};
    reporter.onBuildPredecessors(output);
    return output;
}
