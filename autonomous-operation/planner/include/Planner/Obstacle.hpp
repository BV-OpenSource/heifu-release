#ifndef PLANNERS_OBSTACLE_HPP
#define PLANNERS_OBSTACLE_HPP

#include "Planner/Node.hpp"
#include <gpu_voxels/GpuVoxels.h>

#include <mutex>

using boost::dynamic_pointer_cast;
using boost::shared_ptr;
using gpu_voxels::voxelmap::DistanceVoxelMap;

class Obstacle {
    public:
        Obstacle(){}
        Obstacle(uint64_t address);

        bool isWithinObstacle(Node *node, bool ignoreSafety = false);

        // PRM don't use
        bool edgeWithinObstacle(Node *node1, Node *node2, bool ignoreSafety  = false);

        void updateMapCenter(Vector3f mapCenterMsg);

        std::mutex *mapAccessMutex;

        void updateSafetyDistance(float newDistance);
    private:
        float VoxelSize;
        float safetyDistance;

        std::vector<float> robotSize{0, 0, 0};

        gpu_voxels::DistanceVoxel *deviceDistanceVoxel;
        gpu_voxels::Vector3ui mapDimensions;

        DistanceVoxelMap *pbaDistanceVoxmap;

        Vector3f mapCenter;

        voxelmap::ProbVoxelMap* orig_map;
};

#endif //PLANNERS_OBSTACLE_HPP
