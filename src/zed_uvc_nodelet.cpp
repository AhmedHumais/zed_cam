#include "zed_uvc_nodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(zed_uvc::ZedNodelet, nodelet::Nodelet)

namespace zed_uvc {

    ZedNodelet::~ZedNodelet()
    {
        NODELET_INFO_STREAM("SVO quit");
        zed_->quit_ = true;
    }

    void ZedNodelet::onInit()
    {
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());

        NODELET_INFO_STREAM("Initialized " <<  getName() << " nodelet.");
        zed_.reset(new ZedUVC(nh, pnh));
        if(zed_->init_cap()){
            zed_->start_stream();
        }
    
    }

} // namespace zed_uvc
