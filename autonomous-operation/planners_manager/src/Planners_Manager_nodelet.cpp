#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <Planners_Manager/Planners_Manager.hpp>

namespace Planners_ManagerNodelet
{
	class Planners_ManagerNodelet : public nodelet::Nodelet
	{
		public:
			virtual void onInit()
			{
//				NODELET_INFO("Initializing Planners Manager nodelet ...");
				ros::NodeHandle& nh = this->getNodeHandle();
				ros::NodeHandle& private_nh = this->getPrivateNodeHandle();

				ca_.reset(new PlannersManager(private_nh, nh));
				Planner *Planners = ca_->getPlanner();
				Planners->PrintWorkspaceSize();
				if(Planners != nullptr){
					Planners->Setup();
				}else{
					NODELET_ERROR("Planners not loaded!");
				}
				NODELET_INFO("Initialized Planners Manager nodelet ...");
			}
		private:
			boost::shared_ptr<PlannersManager> ca_;
	};
}

PLUGINLIB_EXPORT_CLASS(Planners_ManagerNodelet::Planners_ManagerNodelet, nodelet::Nodelet)
