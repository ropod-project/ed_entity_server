#ifndef ED_ENTITY_SERVER_ENTITY_SERVER_PLUGIN_H_
#define ED_ENTITY_SERVER_ENTITY_SERVER_PLUGIN_H_

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ed/plugin.h>
#include <ed/types.h>
#include <ropod_ros_msgs/Object.h>
#include <ropod_ros_msgs/ObjectList.h>
#include <ropod_ros_msgs/GetObjectsGoal.h>
#include <ropod_ros_msgs/ToggleObjectPublisher.h>
#include <actionlib/server/simple_action_server.h>
#include <ropod_ros_msgs/GetObjectsAction.h>
//#include <ed_entity_server/GetEntitiesAction.h>
#include <ed_sensor_integration/properties/featureProperties_info.h>



class EntityServerPlugin : public ed::Plugin
{

public:

    EntityServerPlugin();

    ~EntityServerPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

protected:
    ed::PropertyKey<ed::tracking::FeatureProperties> feature_properties;
    // TODO: replace this with ropod_ros_msgs/GetObjects action
    std::shared_ptr<actionlib::SimpleActionServer<ropod_ros_msgs::GetObjectsAction>> get_entities_as;
    ros::ServiceServer toggle_publisher_srv;
    ros::Publisher entities_pub;
    std::vector<ed::EntityConstPtr> entities;
    ros::CallbackQueue cb_queue;

    bool publisher_enabled;
    std::string objects_type;
    geometry_msgs::Polygon objects_area;
    const std::vector<std::string> supported_types = {"rectangles", "circles", "cart_candidates", "carts"};

    /* Cart specific parameters */
    double cart_mobidik_width;
    double cart_mobidik_margin;

    std::vector<ed::EntityConstPtr> filterByArea(const std::vector<ed::EntityConstPtr> &entities, const std::vector<geometry_msgs::Point32> &polygon);
    std::vector<ed::EntityConstPtr> filterRectangles(const std::vector<ed::EntityConstPtr> &entities);
    std::vector<ed::EntityConstPtr> filterCircles(const std::vector<ed::EntityConstPtr> &entities);
    std::vector<ed::EntityConstPtr> getCartCandidates(const std::vector<ed::EntityConstPtr> &entities);
    std::vector<ed::EntityConstPtr> getCarts(const std::vector<ed::EntityConstPtr> &entities);
    void detectMobiDik(const std::vector<ed::EntityConstPtr> &entities, ed::UpdateRequest& req);

    // ROS callbacks
    void getEntitiesCallback(const ropod_ros_msgs::GetObjectsGoalConstPtr &goal);
    bool toggleObjectPublisher(ropod_ros_msgs::ToggleObjectPublisher::Request &req, ropod_ros_msgs::ToggleObjectPublisher::Response &res);
    void publishFilteredEntities();

    void copyEntityToMsg(const ed::EntityConstPtr &e, ropod_ros_msgs::Object &msg);
    std::vector<ed::EntityConstPtr> filterEntities(const std::string &type, const geometry_msgs::Polygon &area);

    bool isEntityInPolygon(const ed::EntityConstPtr &entity, const std::vector<geometry_msgs::Point32> &polygon);
    bool isRectangle(const ed::EntityConstPtr &entity);
    bool isCircle(const ed::EntityConstPtr &entity);
};

#endif
