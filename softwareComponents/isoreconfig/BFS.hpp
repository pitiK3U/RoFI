#include <vector>
#include <set>
#include <map>
#include <legacy/configuration/Configuration.h>
#include <legacy/configuration/Generators.h>
#include <queue>
#include <iomanip>

class BFSReporter
{
    private:
        unsigned long pathLength = 0;
        unsigned long maxQueueSize = 0;
        unsigned long seenCfgs = 0;

    public:
        unsigned long getPathLength() 
        {
            return pathLength;
        }

        unsigned long getMaxQueueSize()
        {
            return maxQueueSize;
        }

        unsigned long getSeenCfgs()
        {
            return seenCfgs;
        }

        void onBuildPredecessors(std::vector<Configuration>& preds)
        {
            pathLength = preds.size();
        }

        void onUpdatePredecessors(std::unordered_map<const Configuration*, const Configuration*>&)
        {
            // pass
        }

        void onUpdateQueue(std::queue<const Configuration*>& queue)
        {
            if (queue.size() > maxQueueSize) 
            {
                maxQueueSize = queue.size();
            }
        }

        void onUpdateSeen(std::unordered_set<Configuration, ConfigurationHash>& seen)
        {
            seenCfgs = seen.size();
        }

        void onUpdateDistance(std::unordered_map<const Configuration*, int>&)
        {
            // pass
            
        }

        void onUpdateCurrent(const Configuration&)
        {
            // pass
        }

        std::string toString()
        {
            std::stringstream out;
            out << std::setw(8) << std::left << "length " << pathLength << std::endl;
            out << std::setw(8) << std::left << "queue " << maxQueueSize << std::endl;
            out << std::setw(8) << std::left << "cfgs " << seenCfgs << std::endl;
            return out.str();
        }
};

std::vector<Configuration> BFS(const Configuration& start, const Configuration& target, 
unsigned int step, unsigned int bound, BFSReporter& reporter);
