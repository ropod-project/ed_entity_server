#ifndef ED_ENTITY_SERVER_ENTITY_SERVER_PLUGIN_H_
#define ED_ENTITY_SERVER_ENTITY_SERVER_PLUGIN_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ed/plugin.h>
#include <ed/types.h>
#include <actionlib/server/simple_action_server.h>
#include <ed_entity_server/GetEntitiesAction.h>
#include <ed_entity_server/GetRectanglesAction.h>
#include <ed_entity_server/GetCirclesAction.h>

class EntityServerPlugin : public ed::Plugin
{

public:

    EntityServerPlugin();

    ~EntityServerPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

protected:

    std::shared_ptr<actionlib::SimpleActionServer<ed_entity_server::GetEntitiesAction>> get_entities_as;
    std::shared_ptr<actionlib::SimpleActionServer<ed_entity_server::GetEntitiesAction>> get_rectangles_as;
    std::shared_ptr<actionlib::SimpleActionServer<ed_entity_server::GetCirclesAction>> get_circles_as;

    ros::ServiceServer switch_publisher_srv;

    ros::Publisher entities_pub;
    ros::Publisher rectangles_pub;
    ros::Publisher circles_pub;

    std::vector<ed::EntityInfo> entities;

    ros::CallbackQueue cb_queue;

    std::vector<ed::EntityInfo> filterByArea(const std::vector<ed::EntityInfo> &entities, const std::vector<geometry_msgs::Point32> &polygon);
    std::vector<ed::EntityInfo> filterRectangles(const std::vector<ed::EntityInfo> &entities);

private:
    void getEntitiesCallback(const ed_entity_server::GetEntitiesGoalConstPtr &goal);
    void getRectanglesCallback(const ed_entity_server::GetEntitiesGoalConstPtr &goal);
    bool isInPolygon(const ed::EntityInfo &entity, const std::vector<geometry_msgs::Point32> &polygon);
    bool isRectangle(const ed::EntityInfo &entity);

};

#endif
