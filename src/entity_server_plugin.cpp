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
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue);
    get_entities_as.reset(
            new actionlib::SimpleActionServer<ed_entity_server::GetEntitiesAction>(
                nh,
                "get_entities",
                boost::bind(&EntityServerPlugin::getEntitiesCallback, this, _1),
                false));
    get_rectangles_as.reset(
            new actionlib::SimpleActionServer<ed_entity_server::GetEntitiesAction>(
                nh,
                "get_rectangles",
                boost::bind(&EntityServerPlugin::getRectanglesCallback, this, _1),
                false));
    get_entities_as->start();
    get_rectangles_as->start();
}

void EntityServerPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    entities.clear();
    const ed::WorldModel& world = data.world;
    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;
        ed::EntityInfo msg;
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
        entities.push_back(msg);
    }
    cb_queue.callAvailable();
}

void EntityServerPlugin::getEntitiesCallback(const ed_entity_server::GetEntitiesGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Received request for entities");
    ed_entity_server::GetEntitiesResult result;
    if (!goal->area.points.empty())
    {
        ROS_INFO_STREAM("...filtering entities by polygon");
        result.entities = filterByArea(entities, goal->area.points);
    }
    else
    {
        result.entities = entities;
    }
    get_entities_as->setSucceeded(result);
}

void EntityServerPlugin::getRectanglesCallback(const ed_entity_server::GetEntitiesGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Received request for rectangular entities");
    ed_entity_server::GetEntitiesResult result;
    result.entities = filterRectangles(entities);
    if (!goal->area.points.empty())
    {
        ROS_INFO_STREAM("...filtering entities by polygon");
        result.entities = filterByArea(result.entities, goal->area.points);
    }
    else
    {
        result.entities = entities;
    }
    get_rectangles_as->setSucceeded(result);
}

std::vector<ed::EntityInfo> EntityServerPlugin::filterByArea(const std::vector<ed::EntityInfo> &entities, const std::vector<geometry_msgs::Point32> &polygon)
{
    std::vector<ed::EntityInfo> filtered_entities;
    for (int i = 0; i < entities.size(); i++)
    {
        if (isInPolygon(entities[i], polygon))
        {
            filtered_entities.push_back(entities[i]);
        }
    }
    return filtered_entities;
}

std::vector<ed::EntityInfo> EntityServerPlugin::filterRectangles(const std::vector<ed::EntityInfo> &entities)
{
    std::vector<ed::EntityInfo> filtered_entities;
    for (int i = 0; i < entities.size(); i++)
    {
        if (isRectangle(entities[i]))
        {
            filtered_entities.push_back(entities[i]);
        }
    }
    return filtered_entities;
}

bool EntityServerPlugin::isInPolygon(const ed::EntityInfo &entity, const std::vector<geometry_msgs::Point32> &polygon)
{
    // TODO: implement this
    return true;
}

bool EntityServerPlugin::isRectangle(const ed::EntityInfo &entity)
{
    // TODO: implement this
    return true;
}

ED_REGISTER_PLUGIN(EntityServerPlugin)
