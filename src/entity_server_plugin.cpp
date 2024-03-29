#include <ed_entity_server/entity_server_plugin.h>
#include <ed_entity_server/utils.h>

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

    tue::Configuration& config = init.config;
    config.value("cart_mobidik_width", cart_mobidik_width, tue::REQUIRED); // [m]
    config.value("cart_mobidik_upper_margin", cart_mobidik_upper_margin, tue::REQUIRED); // [m]
    config.value("cart_mobidik_lower_margin", cart_mobidik_lower_margin, tue::REQUIRED); // [m]

    publisher_enabled = false;

    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue);

    entities_pub = nh.advertise<ropod_ros_msgs::ObjectList>("/ed/objects", 1);
    toggle_publisher_srv = nh.advertiseService("toggle_object_publisher", &EntityServerPlugin::toggleObjectPublisher, this);
    get_entities_as.reset(
            new actionlib::SimpleActionServer<ropod_ros_msgs::GetObjectsAction>(
                nh,
                "/ed/get_objects",
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

    /* Process potential detections. So the below queued queries get fresh results. */
    detectMobiDik(entities, req);

    cb_queue.callAvailable();

    if (publisher_enabled)
    {
        publishFilteredEntities();
    }
}

void EntityServerPlugin::getEntitiesCallback(const ropod_ros_msgs::GetObjectsGoalConstPtr &goal)
{
    ROS_INFO_STREAM("Received request for entities");
    ropod_ros_msgs::GetObjectsResult result;
    std::vector<ed::EntityConstPtr> filtered_entities = filterEntities(goal->type, goal->area);
    for (int i = 0; i < filtered_entities.size(); i++)
    {
        ropod_ros_msgs::Object msg;

        copyEntityToMsg(filtered_entities[i], msg);
        result.objects.push_back(msg);
    }
    get_entities_as->setSucceeded(result);
}

bool EntityServerPlugin::toggleObjectPublisher(ropod_ros_msgs::ToggleObjectPublisher::Request &req, ropod_ros_msgs::ToggleObjectPublisher::Response &res)
{
    if (req.enable_publisher == true)
    {
        if (std::find(supported_types.begin(), supported_types.end(), req.publisher_type) != supported_types.end() ||
           req.publisher_type.empty() ||
           req.publisher_type == "*")
        {
            objects_type = req.publisher_type;
            objects_area = req.area;
            publisher_enabled = true;
            res.success = true;
        }
        else
        {
            res.success = false;
            return true;
        }
    }
    else
    {
        publisher_enabled = false;
        res.success = true;
    }
    return true;
}

void EntityServerPlugin::publishFilteredEntities()
{
    std::vector<ed::EntityConstPtr> filtered_entities = filterEntities(this->objects_type, this->objects_area);
    ropod_ros_msgs::ObjectList object_list;
    for (int i = 0; i < filtered_entities.size(); i++)
    {
        ropod_ros_msgs::Object msg;
        copyEntityToMsg(filtered_entities[i], msg);
        object_list.objects.push_back(msg);
    }
    entities_pub.publish(object_list);
}

// TODO: change this to ropod_ros_msgs/Object message
void EntityServerPlugin::copyEntityToMsg(const ed::EntityConstPtr &e, ropod_ros_msgs::Object &msg)
{
    msg.id = e->id().str();
    msg.type = e->type();
    //msg.existence_probability = e->existenceProbability();

    if (e->has_pose())
    {
        msg.pose.header.frame_id = "map";
        msg.pose.header.stamp = ros::Time::now();
        msg.pose.pose.position.x = e->pose().getOrigin().x;
        msg.pose.pose.position.y = e->pose().getOrigin().y;
        msg.pose.pose.position.z = e->pose().getOrigin().z;
        msg.pose.pose.orientation.x = e->pose().getQuaternion().x;
        msg.pose.pose.orientation.y = e->pose().getQuaternion().y;
        msg.pose.pose.orientation.z = e->pose().getQuaternion().z;
        msg.pose.pose.orientation.w = e->pose().getQuaternion().w;
    }
    else
    {

    }

    if (e->id().str().find("-laserTracking") != std::string::npos)
    {
        ed::tracking::FeatureProperties property = e->property(feature_properties);
        double x, y, w, d, yaw;
        x = property.getRectangle().get_x();
        y = property.getRectangle().get_y();
        w = property.getRectangle().get_w();
        d = property.getRectangle().get_d();
        yaw = property.getRectangle().get_yaw();
        double yaw_top_left = yaw + (M_PI / 4.0);
        double yaw_top_right = yaw - (M_PI / 4.0);
        double yaw_bottom_left = yaw + (3 * M_PI / 4.0);
        double yaw_bottom_right = yaw - (3 * M_PI / 4.0);
        double diagonal_length = std::sqrt(std::pow(w, 2.0) + std::pow(d, 2.0)) / 2.0;
        geometry_msgs::Point32 tl;
        geometry_msgs::Point32 tr;
        geometry_msgs::Point32 bl;
        geometry_msgs::Point32 br;
        tl.x = x + diagonal_length * std::cos(yaw_top_left);
        tl.y = y + diagonal_length * std::sin(yaw_top_left);
        tr.x = x + diagonal_length * std::cos(yaw_top_right);
        tr.y = y + diagonal_length * std::sin(yaw_top_right);
        bl.x = x + diagonal_length * std::cos(yaw_bottom_left);
        bl.y = y + diagonal_length * std::sin(yaw_bottom_left);
        br.x = x + diagonal_length * std::cos(yaw_bottom_right);
        br.y = y + diagonal_length * std::sin(yaw_bottom_right);
        msg.shape.polygon.points.push_back(tr);
        msg.shape.polygon.points.push_back(tl);
        msg.shape.polygon.points.push_back(bl);
        msg.shape.polygon.points.push_back(br);
    }


//    if (!e->shape())
//    {
//        const ed::ConvexHull& ch = e->convexHull();
//
//        if (!ch.points.empty())
//        {
//
//            for(unsigned int i = 0; i < ch.points.size(); ++i)
//            {
//                geometry_msgs::Point p;
//                p.x = ch.points[i].x;
//                p.y = ch.points[i].y;
//                p.z = 0.0;
//                msg.shape.push_back(p);
//            }
//        }
//    }
//
//    std::set<std::string> flags = e->flags();
//    for (std::set<std::string>::iterator it = flags.begin(); it != flags.end(); ++it)
//    {
//        msg.flags.push_back(*it);
//    }
//
//    std::set<ed::TYPE> types = e->types();
//    for (std::set<ed::TYPE>::iterator it = types.begin(); it != types.end(); ++it)
//    {
//        msg.types.push_back(*it);
//    }
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
    for (int i = 0; i < entities.size(); i++)
    {
		const ed::EntityConstPtr& e = entities[i];
		if(e->hasFlag("cart"))
		{
			carts.push_back(e);
		}
    }
    return carts;
}

bool EntityServerPlugin::isEntityInPolygon(const ed::EntityConstPtr &entity, const std::vector<geometry_msgs::Point32> &polygon)
{
    if (entity->id().str().find("-laserTracking") == std::string::npos)
    {
        return false;
    }
    ed::tracking::FeatureProperties property = entity->property(feature_properties);
    geometry_msgs::Point32 entity_center;
    entity_center.x = property.getRectangle().get_x();
    entity_center.y = property.getRectangle().get_y();
    // defined in utils.h
    return isPointInPolygon(polygon, entity_center);
}

bool EntityServerPlugin::isRectangle(const ed::EntityConstPtr &entity)
{
    if (entity->id().str().find("-laserTracking") == std::string::npos)
    {
        return false;
    }
    ed::tracking::FeatureProperties property = entity->property(feature_properties);
    if (property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle())
    {
        return true;
    }
    return false;
}

bool EntityServerPlugin::isCircle(const ed::EntityConstPtr &entity)
{
    if (entity->id().str().find("-laserTracking") == std::string::npos)
    {
        return false;
    }
    ed::tracking::FeatureProperties property = entity->property(feature_properties);
    if (property.getFeatureProbabilities().get_pCircle() > property.getFeatureProbabilities().get_pRectangle())
    {
        return true;
    }
    return false;
}


void EntityServerPlugin::detectMobiDik(const std::vector<ed::EntityConstPtr> &entities, ed::UpdateRequest& req)
{
    for (int i = 0; i < entities.size(); i++)
    {
		const ed::EntityConstPtr& e = entities[i];

    	/* Pre-filtering since we only need to evaluate entities, generated by the  "laser tracking"  plug-in */
		std::string laserID = "-laserTracking";
		if (e->id().str().length() < laserID.length())
		{
			continue;
		}
		if (e->id().str().substr ( e->id().str().length() - laserID.size() ) != laserID)
		{
			continue;
		}

    	/* Detect Mobidik by its dimensions */
    	ed::tracking::FeatureProperties property = e->property (feature_properties);
		if ( property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle() && // Dimension check
				property.rectangle_.get_d() < cart_mobidik_width + cart_mobidik_upper_margin &&
				property.rectangle_.get_w() < cart_mobidik_width + cart_mobidik_upper_margin &&
				( property.rectangle_.get_d() > cart_mobidik_width - cart_mobidik_lower_margin ||
				  property.rectangle_.get_w() > cart_mobidik_width - cart_mobidik_lower_margin ))
		{
			ROS_DEBUG_STREAM ("Mobidik found for entity = " << e->id() << std::endl);
			/* NOTE: This "Mobidik" flag is important for visualization.
			 * Thus, it must be processes independent of queries. */
			req.setFlag(e->id(), "cart"); //TODO check if it exists already?
		}
    }
}

ED_REGISTER_PLUGIN(EntityServerPlugin)
