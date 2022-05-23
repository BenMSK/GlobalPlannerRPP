#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "../include/GlobalPlanner.h"

namespace globalplanner {
    class GlobalPlannerNodelet: public nodelet::Nodelet {
    public:
        GlobalPlannerNodelet() {}
        ~GlobalPlannerNodelet() {}

        void onInit(void) {
            GlobalPlanner_.reset(new GlobalPlanner(getNodeHandle(), getPrivateNodeHandle()));
        }

    private:

        boost::shared_ptr<GlobalPlanner> GlobalPlanner_;
    };
};

PLUGINLIB_EXPORT_CLASS(globalplanner::GlobalPlannerNodelet, nodelet::Nodelet);