#include "BFS.h"

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

        void onUpdateQueue(std::queue<const Configuration*>* queue)
        {
            
        }
}
