#include <ed_entity_server/entity_server_plugin.h>

#include <ed/entity.h>
#include <ed/world_model.h>
#include <ed/update_request.h>


EntityServerPlugin::EntityServerPlugin()
{
}

EntityServerPlugin::~EntityServerPlugin()
{
}

void EntityServerPlugin::initialize(ed::InitData& init)
{
    init.properties.registerProperty("Feature", feature_properties, new FeaturPropertiesInfo);
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue);
    toggle_publisher_srv = nh.advertiseService("toggle_object_publisher", &EntityServerPlugin::toggleObjectPublisher, this);
    get_entities_as.reset(
            new actionlib::SimpleActionServer<ed_entity_server::GetEntitiesAction>(
                nh,
                "get_entities",
                boost::bind(&EntityServerPlugin::getEntitiesCallback, this, _1),
                false));
    get_entities_as->start();
}

void EntityServerPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    entities.clear();
    const ed::WorldModel& world = data.world;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        entities.push_back(e);
    }

    cb_queue.callAvailable();

    if (publisher_enabled)
    {
        publishFilteredEntities();
    }
}

void EntityServerPlugin::getEntitiesCallback(const ed_entity_server::GetEntitiesGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Received request for entities");
    ed_entity_server::GetEntitiesResult result;
    std::vector<ed::EntityConstPtr> filtered_entities = filterEntities(goal->type, goal->area);
    for (int i = 0; i < filtered_entities.size(); i++)
    {
        ed::EntityInfo msg;
        copyEntityToMsg(filtered_entities[i], msg);
        result.entities.push_back(msg);
    }
    get_entities_as->setSucceeded(result);
}

bool EntityServerPlugin::toggleObjectPublisher(ropod_ros_msgs::ToggleObjectPublisher::Request &req, ropod_ros_msgs::ToggleObjectPublisher::Response &res)
{
    if (req.enable_publisher == true)
    {
        if (std::find(supported_types.begin(), supported_types.end(), req.publisher_type) != supported_types.end())
        {
            objects_type = req.publisher_type;
            objects_area = req.area;
            publisher_enabled = true;
            res.success = true;
        }
        else
        {
            return res.success = false;
        }
    }
    else
    {
        publisher_enabled = false;
        res.success = true;
    }
}

void EntityServerPlugin::publishFilteredEntities()
{
    std::vector<ed::EntityConstPtr> filtered_entities = filterEntities(this->objects_type, this->objects_area);
}

// TODO: change this to ropod_ros_msgs/Object message
void EntityServerPlugin::copyEntityToMsg(const ed::EntityConstPtr &e, ed::EntityInfo &msg)
{
    msg.id = e->id().str();
    msg.type = e->type();
    msg.existence_probability = e->existenceProbability();

    if (e->has_pose())
    {
        msg.pose.position.x = e->pose().getOrigin().x;
        msg.pose.position.y = e->pose().getOrigin().y;
        msg.pose.position.z = e->pose().getOrigin().z;
        msg.pose.orientation.x = e->pose().getQuaternion().x;
        msg.pose.orientation.y = e->pose().getQuaternion().y;
        msg.pose.orientation.z = e->pose().getQuaternion().z;
        msg.pose.orientation.w = e->pose().getQuaternion().w;
        msg.has_pose = true;
    }
    else
    {
        msg.has_pose = false;
    }

    if (!e->shape())
    {
        const ed::ConvexHull& ch = e->convexHull();

        if (!ch.points.empty())
        {
            msg.z_min = ch.z_min;
            msg.z_max = ch.z_max;

            unsigned int size = ch.points.size();

            for(unsigned int i = 0; i < size; ++i)
            {
                geometry_msgs::Point p;
                p.x = ch.points[i].x;
                p.y = ch.points[i].y;
                p.z = 0.0;
                msg.convex_hull.push_back(p);
            }
        }
    }

    std::set<std::string> flags = e->flags();
    for (std::set<std::string>::iterator it = flags.begin(); it != flags.end(); ++it)
    {
        msg.flags.push_back(*it);
    }

    std::set<ed::TYPE> types = e->types();
    for (std::set<ed::TYPE>::iterator it = types.begin(); it != types.end(); ++it)
    {
        msg.types.push_back(*it);
    }
}

std::vector<ed::EntityConstPtr> EntityServerPlugin::filterEntities(const std::string &type, const geometry_msgs::Polygon &area)
{
    std::vector<ed::EntityConstPtr> filtered_entities;
    if (type.empty())
    {
        filtered_entities = entities;
    }
    else if (type == "rectangles")
    {
        ROS_DEBUG_STREAM("..returning rectangular entities");
        filtered_entities = filterRectangles(entities);
    }
    else if (type == "circles")
    {
        ROS_DEBUG_STREAM("..returning circular entities");
        filtered_entities = filterCircles(entities);
    }
    else if (type == "cart_candidates")
    {
        ROS_DEBUG_STREAM("..returning cart candidates");
        filtered_entities = getCartCandidates(entities);
    }
    else if (type == "carts")
    {
        ROS_DEBUG_STREAM("..returning carts");
        filtered_entities = getCarts(entities);
    }
    else
    {
        filtered_entities = entities;
    }
    if (!area.points.empty())
    {
        ROS_DEBUG_STREAM("...filtering entities by area");
        filtered_entities = filterByArea(filtered_entities, area.points);
    }
    return filtered_entities;
}


std::vector<ed::EntityConstPtr> EntityServerPlugin::filterByArea(const std::vector<ed::EntityConstPtr> &entities, const std::vector<geometry_msgs::Point32> &polygon)
{
    std::vector<ed::EntityConstPtr> filtered_entities;
    for (int i = 0; i < entities.size(); i++)
    {
        if (isEntityInPolygon(entities[i], polygon))
        {
            filtered_entities.push_back(entities[i]);
        }
    }
    return filtered_entities;
}

std::vector<ed::EntityConstPtr> EntityServerPlugin::filterRectangles(const std::vector<ed::EntityConstPtr> &entities)
{
    std::vector<ed::EntityConstPtr> filtered_entities;
    for (int i = 0; i < entities.size(); i++)
    {
        if (isRectangle(entities[i]))
        {
            filtered_entities.push_back(entities[i]);
        }
    }
    return filtered_entities;
}

std::vector<ed::EntityConstPtr> EntityServerPlugin::filterCircles(const std::vector<ed::EntityConstPtr> &entities)
{
    std::vector<ed::EntityConstPtr> filtered_entities;
    for (int i = 0; i < entities.size(); i++)
    {
        if (isCircle(entities[i]))
        {
            filtered_entities.push_back(entities[i]);
        }
    }
    return filtered_entities;
}

std::vector<ed::EntityConstPtr> EntityServerPlugin::getCartCandidates(const std::vector<ed::EntityConstPtr> &entities)
{
    std::vector<ed::EntityConstPtr> cart_candidates;
    return cart_candidates;
}

std::vector<ed::EntityConstPtr> EntityServerPlugin::getCarts(const std::vector<ed::EntityConstPtr> &entities)
{
    std::vector<ed::EntityConstPtr> carts;
    return carts;
}

bool EntityServerPlugin::isEntityInPolygon(const ed::EntityConstPtr &entity, const std::vector<geometry_msgs::Point32> &polygon)
{
    // TODO: implement this
    return true;
}

bool EntityServerPlugin::isRectangle(const ed::EntityConstPtr &entity)
{
    ed::tracking::FeatureProperties property = entity->property(feature_properties);
    if (property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle())
    {
        return true;
    }
    return false;
}

bool EntityServerPlugin::isCircle(const ed::EntityConstPtr &entity)
{
    ed::tracking::FeatureProperties property = entity->property(feature_properties);
    if (property.getFeatureProbabilities().get_pCircle() > property.getFeatureProbabilities().get_pRectangle())
    {
        return true;
    }
    return false;
}

ED_REGISTER_PLUGIN(EntityServerPlugin)
