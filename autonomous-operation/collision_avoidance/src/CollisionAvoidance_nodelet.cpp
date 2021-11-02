#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "CollisionAvoidance/CollisionAvoidance.hpp"

namespace CollisionAvoidanceNodelet
{
    class CollisionAvoidanceNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit()
            {
                NODELET_INFO("Initializing Collision Avoidance nodelet ...");
                ros::NodeHandle& nh = this->getNodeHandle();
                ros::NodeHandle& private_nh = this->getPrivateNodeHandle();

                ca_.reset(new CollisionAvoidance(private_nh, nh));
            }
        private:
            boost::shared_ptr<CollisionAvoidance> ca_;
    };
}

PLUGINLIB_EXPORT_CLASS(CollisionAvoidanceNodelet::CollisionAvoidanceNodelet, nodelet::Nodelet)
