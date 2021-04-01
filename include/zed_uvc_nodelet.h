#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include "zed_cam_uvc.hpp"

namespace zed_uvc{

    class ZedNodelet : public nodelet::Nodelet{
    public:
        ZedNodelet() = default;
        virtual ~ZedNodelet();

    private:
        virtual void onInit();
        
        std::unique_ptr<ZedUVC> zed_;
    };

}
