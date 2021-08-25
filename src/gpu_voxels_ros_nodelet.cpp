#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <GpuVoxelsRos/GpuVoxelsRos.hpp>

namespace GPUVoxelsROSNodelet
{
    class GPUVoxelsROSNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit()
            {
                NODELET_INFO("Initializing GPU Voxels nodelet ...");
                ros::NodeHandle& nh = this->getNodeHandle();
                ros::NodeHandle& private_nh = this->getPrivateNodeHandle();

                ca_.reset(new GPUVoxelsROS(private_nh, nh));
                NODELET_INFO("Setting Up GPU Voxels nodelet ...");
                ca_->Setup();
            }
        private:
            boost::shared_ptr<GPUVoxelsROS> ca_;
    };
}

PLUGINLIB_EXPORT_CLASS(GPUVoxelsROSNodelet::GPUVoxelsROSNodelet, nodelet::Nodelet)
