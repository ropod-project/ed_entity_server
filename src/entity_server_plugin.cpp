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

    tue::Configuration& config = init.config;
    config.value("cart_mobidik_width", cart_mobidik_width, tue::REQUIRED); // [m]
    config.value("cart_mobidik_margin", cart_mobidik_margin, tue::REQUIRED); // [m]

    publisher_enabled = false;

    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue);

    entities_pub = nh.advertise<ropod_ros_msgs::ObjectList>("/ed/objects", 1);
    toggle_publisher_srv = nh.advertiseService("toggle_object_publisher", &EntityServerPlugin::toggleObjectPublisher, this);
    get_entities_as.reset(
            new actionlib::SimpleActionServer<ropod_ros_msgs::GetObjectsAction>(
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
		if(e->hasFlag("Mobidik"))
		{
			carts.push_back(e);
		}
    }
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
	    ed::PropertyKey<ed::tracking::FeatureProperties> featureProperties;
    	ed::tracking::FeatureProperties property = e->property ( featureProperties );
		if ( property.getFeatureProbabilities().get_pRectangle() > property.getFeatureProbabilities().get_pCircle() && // Dimension check
				property.rectangle_.get_d() < cart_mobidik_width + cart_mobidik_margin &&
				property.rectangle_.get_w() < cart_mobidik_width + cart_mobidik_margin &&
				( property.rectangle_.get_d() > cart_mobidik_width - cart_mobidik_margin ||
				  property.rectangle_.get_w() > cart_mobidik_width - cart_mobidik_margin ))
		{
			ROS_DEBUG_STREAM ("Mobidik found for entity = " << e->id() << std::endl);
			/* NOTE: This "Mobidik" flag is important for visualization.
			 * Thus, it must be processes independent of queries. */
			req.setFlag(e->id(), "Mobidik"); //TODO check if it exists already?
		}
    }
}

ED_REGISTER_PLUGIN(EntityServerPlugin)
