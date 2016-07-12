/**

   Author : Anirudh Vemula
   Date : 12 March 2016

   Taken from Gochev's code for path planning with adaptive dimensionality

*/


#include <sbpl_dynamic_adaptive_planner/navviz.h>

using namespace std;
#define TWO_PI 6.2831853


void HSVtoRGB( double *r, double *g, double *b, double h, double s, double v )
{
  int i;
  double f, p, q, t;
  if( s == 0 ) {
    // achromatic (grey)
    *r = *g = *b = v;
    return;
  }
  h /= 60;        // sector 0 to 5
  i = floor(h);
  f = h - i;			// factorial part of h
  p = v * ( 1 - s );
  q = v * ( 1 - s * f );
  t = v * ( 1 - s * ( 1 - f ) );
  switch( i ) {
  case 0:
    *r = v;
    *g = t;
    *b = p;
    break;
  case 1:
    *r = q;
    *g = v;
    *b = p;
    break;
  case 2:
    *r = p;
    *g = v;
    *b = t;
    break;
  case 3:
    *r = p;
    *g = q;
    *b = v;
    break;
  case 4:
    *r = t;
    *g = p;
    *b = v;
    break;
  default:
    *r = v;
    *g = p;
    *b = q;
    break;
  }
}

void NavVisualizer::advertiseTransform(){
  //return;
  tf::Transform transform;

  transform.setOrigin( tf::Vector3(offsetX, offsetY, offsetZ) );
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "navviz"));
}

NavVisualizer::NavVisualizer(std::string robot_name, double robotW, double robotH, double offX, double offY, double offZ): ph_("~")
{
  robot_width_ = robotW;
  robot_height_ = robotH;
  offsetX = offX;
  offsetY = offY;
  offsetZ = offZ;
  num_joints_ = 3;
  reference_frame_ = "navviz";

  srand (time(NULL));

  joint_names_.push_back("_pos_x");
  joint_names_.push_back("_pos_y");
  joint_names_.push_back("_theta");

  link_names_.push_back("_x_link");
  link_names_.push_back("_y_link");
  link_names_.push_back("_theta_link");

  //meshes
  meshes_.push_back("package://pr2_description/meshes/shoulder_v0/shoulder_yaw.stl");
  
  marker_array_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 500);
  marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 500);
  //display_trajectory_publisher_ = nh_.advertise<motion_planning_msgs::DisplayTrajectory>("nav_viz", 200);
  
  /*printf("Waiting for subscribers for marker_array_publisher"); fflush(stdout);
    while(marker_array_publisher_.getNumSubscribers() < 1 && nh_.ok())
    {
    printf(".");
    ros::Duration(1.0).sleep();
    }
    printf("marker_array_publisher has %d subscribers [ros handle ok? %d]", marker_array_publisher_.getNumSubscribers(), nh_.ok());
  
    printf("Waiting for subscribers for marker_publisher"); fflush(stdout);
    while(marker_publisher_.getNumSubscribers() < 1 && nh_.ok())
    {
    printf(".");
    ros::Duration(1.0).sleep();
    }
    printf("marker_publisher has %d subscribers [ros handle ok? %d]", marker_publisher_.getNumSubscribers(), nh_.ok());*/
  advertiseTransform();
}

NavVisualizer::~NavVisualizer()
{

}

void NavVisualizer::setReferenceFrame(std::string &frame)
{
  reference_frame_=frame;
}

void NavVisualizer::visualizeSimpleObstacles(const std::vector<std::vector<double> > &obstacles, std::string *objName, int color, double trans)
{
  advertiseTransform();
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);
  
  marker_array_.markers.clear();
  marker_array_.markers.resize(1);

  //ROS_INFO("Displaying %d simple obstacles in the %s frame", (int)obstacles.size(), reference_frame_.c_str());

  std::string ns;
  if(objName == NULL){
    ns = "obstacles"+boost::lexical_cast<std::string>(rand());
  } else {
    ns = *objName;
  }

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = ns;
  marker.id = rand();
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0; //obstacles[i][0];
  marker.pose.position.y = 0.0; //obstacles[i][1];
  marker.pose.position.z = 0.0; //obstacles[i][2];
  marker.scale.x = obstacles[0][3];
  marker.scale.y = obstacles[0][4];
  marker.scale.z = obstacles[0][5];
  marker.points.resize(obstacles.size());
  marker.color.r = r;//0.0;
  marker.color.g = g;//0.0;
  marker.color.b = b;//0.5;
  marker.color.a = trans;

  for(int i = 0; i < int(obstacles.size()); i++)
    {
      marker.points[i].x = obstacles[i][0];
      marker.points[i].y = obstacles[i][1];
      marker.points[i].z = obstacles[i][2];
    }
  marker.lifetime = ros::Duration(600.0);

  marker_publisher_.publish(marker);
}

void NavVisualizer::visualizeComplexObstacles(const std::vector<std::vector<double> > &obstacles, std::string *objName, int color, double trans)
{
  advertiseTransform();
  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);
  
  marker_array_.markers.clear();
  marker_array_.markers.resize(obstacles.size());

  //ROS_INFO("Displaying %d obstacles in the %s frame", (int)obstacles.size(), reference_frame_.c_str());

  std::string ns;
  if(objName == NULL){
    ns = "obstacles"+boost::lexical_cast<std::string>(rand());
  } else {
    ns = *objName;
  }

  for(int i = 0; i < int(obstacles.size()); i++)
    {
      if(obstacles[i].size() < 6)
	{
	  ROS_INFO("Obstacle description doesn't have length = 6");
	  continue;
	}

      //TODO: Change this to use a CUBE_LIST
      marker_array_.markers[i].header.stamp = ros::Time::now();
      marker_array_.markers[i].header.frame_id = reference_frame_;
      marker_array_.markers[i].ns = ns;
      marker_array_.markers[i].id = rand();
      marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array_.markers[i].pose.position.x = obstacles[i][0];
      marker_array_.markers[i].pose.position.y = obstacles[i][1];
      marker_array_.markers[i].pose.position.z = obstacles[i][2];
      marker_array_.markers[i].scale.x = obstacles[i][3];
      marker_array_.markers[i].scale.y = obstacles[i][4];
      marker_array_.markers[i].scale.z = obstacles[i][5];
      marker_array_.markers[i].color.r = r;//0.0;
      marker_array_.markers[i].color.g = g;//0.0;
      marker_array_.markers[i].color.b = b;//0.5;
      marker_array_.markers[i].color.a = trans;
      marker_array_.markers[i].lifetime = ros::Duration(600.0);
    }

  marker_array_publisher_.publish(marker_array_);
}

void NavVisualizer::visualizeConfigurations(std::string name, double scolor, double ecolor, const std::vector<std::vector<double> > &traj, int throttle)
{
  advertiseTransform();
  double color_inc = (ecolor - scolor) / traj.size();   //260 is blue
  
  std::vector<geometry_msgs::PoseStamped> poses;
  std::vector<double> colors;
  geometry_msgs::PoseStamped pose;
  
  //update the poses to the new frame
  ROS_WARN("Visualizing %d configurations!", (int)traj.size());
  for(unsigned int i = 0; i < traj.size(); i+=throttle){  
    pose.pose.position.x = traj[i][0];
    pose.pose.position.y = traj[i][1];
    pose.pose.position.z = 0.0f;

    pose.pose.orientation.w = cos((traj[i][2] + (TWO_PI / 4.0)) / 2.0);
    pose.pose.orientation.x = 0;
    pose.pose.orientation.y = 0;
    pose.pose.orientation.z = sin((traj[i][2] + (TWO_PI / 4.0)) / 2.0);
    poses.push_back(pose);
    colors.push_back(scolor + i * color_inc);
  }

  visualizeMeshes(colors, poses, name);
}

void NavVisualizer::visualizeCircle(std::vector<double> pose, int color, std::string text, double radius)
{
  advertiseTransform();
  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = text;
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::LINE_STRIP;
  obs_marker.action = 0;
  obs_marker.scale.x = 0.05;
  obs_marker.scale.y = 0.05;
  obs_marker.scale.z = 0.025;
  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);
  obs_marker.color.r = r;
  obs_marker.color.g = g;
  obs_marker.color.b = b;
  obs_marker.color.a = 1.0;
  obs_marker.lifetime = ros::Duration(600.0);

  obs_marker.points.resize(31);
  obs_marker.colors.resize(31);
  
  double angle_step = TWO_PI / 30.0f;
  for (int k = 0; k < 31; k++)
    {
      obs_marker.points[k].x = pose[0] + radius * cos(k * angle_step);
      obs_marker.points[k].y = pose[1] + radius * sin(k * angle_step);
      obs_marker.points[k].z = 0.02;
      obs_marker.colors[k].r = r;
      obs_marker.colors[k].g = g;
      obs_marker.colors[k].b = b;
      obs_marker.colors[k].a = 1.0f;
    }

  marker_publisher_.publish(obs_marker);
}

void NavVisualizer::visualizeSphere(std::vector<double> pose, int color, std::string text, double radius)
{
  advertiseTransform();
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text + "-sphere";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = pose[0];
  marker.pose.position.y = pose[1];
  marker.pose.position.z = pose[2];
  marker.scale.x = radius;
  marker.scale.y = radius;
  marker.scale.z = radius;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(600.0);

  marker_publisher_.publish(marker);
}

void NavVisualizer::visualizeCylinder(std::vector<double> p1, std::vector<double> p2, int color, std::string text, double radius)
{
  advertiseTransform();
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text + "-cylinder";
  marker.id = 1;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = p1[0];
  marker.pose.position.y = p1[1];
  marker.pose.position.z = p1[2];
  marker.scale.x = radius*2;
  marker.scale.y = radius*2;
  marker.scale.z = radius*2;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(600.0);

  marker_publisher_.publish(marker);
}

void NavVisualizer::visualizePoints(const std::vector<std::vector<double> > &pose, int color, std::string text)
{
  advertiseTransform();
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(120.0);
  marker.id = 1;

  marker.points.resize(pose.size());
  for(size_t i = 0; i < pose.size(); i++)
    {
      marker.points[i].x = pose[i][0];
      marker.points[i].y = pose[i][1];
      marker.points[i].z = pose[i][2];
    }

  marker_publisher_.publish(marker);
}

void NavVisualizer::visualizePoints(const std::vector<std::vector<double> > &pose, const std::vector<double> &colors, std::string text)
{
  advertiseTransform();
  if(colors.size() != pose.size()){
    printf("Bad colors array size %d (expected %d)\n", (int) colors.size(), (int) pose.size());
    return;
  }
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = text;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.025;
  marker.scale.y = 0.025;
  marker.scale.z = 0.025;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(120.0);
  marker.id = 1;

  marker.points.resize(pose.size());
  marker.colors.resize(colors.size());
  for(size_t i = 0; i < pose.size(); i++)
    {
      marker.points[i].x = pose[i][0];
      marker.points[i].y = pose[i][1];
      marker.points[i].z = pose[i][2];
      HSVtoRGB(&r, &g, &b, colors[i], 1.0, 1.0);
      marker.colors[i].r = r;
      marker.colors[i].g = g;
      marker.colors[i].b = b;
      marker.colors[i].a = 1.0f;
    }

  marker_publisher_.publish(marker);
}

void NavVisualizer::visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius)
{
  advertiseTransform();
  double r=0,g=0,b=0;
  visualization_msgs::Marker marker;

  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);

  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;
  marker.ns = "spheres-" + text;
  marker.type = visualization_msgs::Marker::SPHERE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = radius*2.0;
  marker.scale.y = radius*2.0;
  marker.scale.z = radius*2.0;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(120.0);
  marker.id = 1;

  marker.points.resize(pose.size());
  for(size_t i = 0; i < pose.size(); i++)
    {
      marker.points[i].x = pose[i][0];
      marker.points[i].y = pose[i][1];
      marker.points[i].z = pose[i][2];
    }

  marker_publisher_.publish(marker);
}

void NavVisualizer::visualizeMesh(double color_num, std::vector<geometry_msgs::PoseStamped> &poses, std::string name)
{
  advertiseTransform();
  //ROS_WARN("visualizeMeshes not complete yet! Just visualizing a CUBE");
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(poses.size());
  ros::Time time = ros::Time::now();

  HSVtoRGB(&r, &g, &b, color_num, 1.0, 1.0);

  
  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
    {
      marker_array_.markers[i].header.stamp = time;
      marker_array_.markers[i].header.frame_id = reference_frame_;
      marker_array_.markers[i].ns = name;
      //marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
      marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
      marker_array_.markers[i].id = i;
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array_.markers[i].pose = poses.at(i).pose;
      marker_array_.markers[i].pose.position.z = 0.20;
    
      marker_array_.markers[i].scale.x = robot_width_;  
      marker_array_.markers[i].scale.y = robot_height_;
      marker_array_.markers[i].scale.z = 0.40;

      marker_array_.markers[i].color.r = r;
      marker_array_.markers[i].color.g = g;
      marker_array_.markers[i].color.b = b;
      marker_array_.markers[i].color.a = 1.0;
      marker_array_.markers[i].lifetime = ros::Duration(600.0);
      //marker_array_.markers[i].mesh_resource = meshes_[i];
    }

  marker_array_publisher_.publish(marker_array_);
}

void NavVisualizer::visualizeMeshes(const std::vector<double> &colors, std::vector<geometry_msgs::PoseStamped> &poses, std::string name)
{
  advertiseTransform();
  //ROS_WARN("visualizeMeshes not complete yet! Just visualizing a CUBE");
  double r,g,b;
  marker_array_.markers.clear();
  marker_array_.markers.resize(poses.size());
  ros::Time time = ros::Time::now();

  
  for(int i = 0; i < (int)marker_array_.markers.size(); ++i)
    {
      marker_array_.markers[i].header.stamp = time;
      marker_array_.markers[i].header.frame_id = reference_frame_;
      marker_array_.markers[i].ns = name;
      marker_array_.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
      //marker_array_.markers[i].type = visualization_msgs::Marker::CUBE;
      marker_array_.markers[i].id = i;
      marker_array_.markers[i].action = visualization_msgs::Marker::ADD;
      marker_array_.markers[i].pose = poses.at(i).pose;
      marker_array_.markers[i].pose.position.z = 0.20;
    
      marker_array_.markers[i].scale.x = robot_width_;  
      marker_array_.markers[i].scale.y = robot_height_;
      marker_array_.markers[i].scale.z = 0.40;

      HSVtoRGB(&r, &g, &b, colors[i], 1.0, 1.0);
    
      marker_array_.markers[i].color.r = r;
      marker_array_.markers[i].color.g = g;
      marker_array_.markers[i].color.b = b;
      marker_array_.markers[i].color.a = 1.0;
      marker_array_.markers[i].lifetime = ros::Duration(600.0);
      //marker_array_.markers[i].mesh_resource = meshes_[i];
    }

  marker_array_publisher_.publish(marker_array_);
}

void NavVisualizer::visualizeConfiguration(double color_num, const std::vector<double> &coords, std::string name)
{
  advertiseTransform();
  
  std::vector<geometry_msgs::PoseStamped> poses;
  poses.resize(1);
  geometry_msgs::PoseStamped pose;
  
  //update the poses to the new frame
  for(unsigned int i = 0; i < poses.size(); i++){  
    poses[i].pose.position.x = coords[0];
    poses[i].pose.position.y = coords[1];
    poses[i].pose.position.z = 0.0f;

    poses[i].pose.orientation.w = cos((coords[2] + (TWO_PI / 4.0)) / 2.0);
    poses[i].pose.orientation.x = 0;
    poses[i].pose.orientation.y = 0;
    poses[i].pose.orientation.z = sin((coords[2] + (TWO_PI / 4.0)) / 2.0);
  }

  visualizeMesh(color_num, poses, name);
}

void NavVisualizer::visualizeLine(std::vector<double> p1, std::vector<double> p2, int color, std::string name){
  advertiseTransform();
  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = name;
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::LINE_STRIP;
  obs_marker.action = 0;
  obs_marker.scale.x = robot_width_ / 1.0;
  obs_marker.scale.y = robot_width_ / 1.0;
  obs_marker.scale.z = 0.025;
  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);
  obs_marker.color.r = r;
  obs_marker.color.g = g;
  obs_marker.color.b = b;
  obs_marker.color.a = 1.0;
  obs_marker.lifetime = ros::Duration(120.0);

  obs_marker.points.resize(2);
  obs_marker.colors.resize(2);
    
  obs_marker.points[0].x = p1[0];
  obs_marker.points[0].y = p1[1];
  obs_marker.points[0].z = p1[2];
  obs_marker.colors[0].r = r;
  obs_marker.colors[0].g = g;
  obs_marker.colors[0].b = b;
  obs_marker.colors[0].a = 1.0f;    

  obs_marker.points[1].x = p2[0];
  obs_marker.points[1].y = p2[1];
  obs_marker.points[1].z = p2[2];
  obs_marker.colors[1].r = r;
  obs_marker.colors[1].g = g;
  obs_marker.colors[1].b = b;
  obs_marker.colors[1].a = 1.0f;

  marker_publisher_.publish(obs_marker);
}

void NavVisualizer::visualizePath(std::vector<std::vector<double> > &dpath, int scolor, int ecolor, std::string ns)
{
  advertiseTransform();
  if(dpath.empty())
    {
      //ROS_INFO("[visualizeShortestPath] The shortest path is empty.");
      return;
    }
  //else
  //  ROS_INFO("[visualizeShortestPath] There are %i waypoints in the shortest path.",int(dpath.size()));

  visualization_msgs::Marker obs_marker;
  obs_marker.header.frame_id = reference_frame_;
  obs_marker.header.stamp = ros::Time();
  obs_marker.header.seq = 0;
  obs_marker.ns = ns;// + "path_" + boost::lexical_cast<std::string>(scolor) + "_" + boost::lexical_cast<std::string>(ecolor);
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::Marker::LINE_STRIP;
  obs_marker.action = 0;
  obs_marker.scale.x = robot_width_ / 1.0;
  obs_marker.scale.y = robot_width_ / 1.0;
  obs_marker.scale.z = 0.025;
  double r=0,g=0,b=0;
  HSVtoRGB(&r, &g, &b, scolor, 1.0, 1.0);
  obs_marker.color.r = r;
  obs_marker.color.g = g;
  obs_marker.color.b = b;
  obs_marker.color.a = 1.0;
  obs_marker.lifetime = ros::Duration(120.0);

  obs_marker.points.resize(dpath.size());
  obs_marker.colors.resize(dpath.size());
  
  
  float color_step = (ecolor - scolor) / ((float) dpath.size());
  for (int k = 0; k < int(dpath.size()); k++)
    {
      if(int(dpath[k].size()) < 2)
	continue;

      obs_marker.points[k].x = dpath[k][0];
      obs_marker.points[k].y = dpath[k][1];
      obs_marker.points[k].z = 0.0;
      double color = scolor; //+ (double) k * color_step;
      HSVtoRGB(&r, &g, &b, color, 1.0, 1.0);
      obs_marker.colors[k].r = r;
      obs_marker.colors[k].g = g;
      obs_marker.colors[k].b = b;
      obs_marker.colors[k].a = 1.0f;
    }

  marker_publisher_.publish(obs_marker);
}

void NavVisualizer::visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size)
{
  advertiseTransform();
  unsigned int inc = 1;
  visualization_msgs::Marker marker;
  
  //check if the list is empty
  if(states.empty())
    {
      ROS_DEBUG("[visualizeBasicStates] There are no states in the %s states list.", name.c_str());
      return;
    }

  //if there are too many states, rviz will crash and burn when drawing
  if(states.size() > 50000)
    inc = 4;
  else if(states.size() > 10000)
    inc = 2;
  else
    inc = 1;

  marker.points.resize(states.size()/inc + 1);

  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = reference_frame_;

  marker.ns = name;
  marker.id = 1;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration(120.0);

  unsigned int m_ind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
    {
      if(states[i].size() >= 3)
	{
	  marker.points[m_ind].x = states[i][0];
	  marker.points[m_ind].y = states[i][1];
	  marker.points[m_ind].z = states[i][2];
	  ++m_ind;
	}
    }

  marker_publisher_.publish(marker);
  ROS_DEBUG("[visualizeBasicStates] published %d markers for %s states", int(marker.points.size()), name.c_str());
}

void NavVisualizer::visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size)
{
  advertiseTransform();
  unsigned int inc = 1;
  std::vector<double> scaled_color(4,0);
  visualization_msgs::MarkerArray marker_array;

  //check if the list is empty
  if(states.empty())
    {
      ROS_INFO("[visualizeDetailedStates] There are no states in the %s states list", name.c_str());
      return;
    } 
  else
    ROS_INFO("[visualizeDetailedStates] There are %i states in the %s states list.",int(states.size()),name.c_str());
    
  if(color.size()<2)
    {
      ROS_INFO("[visualizeDetailedStates] Not enough colors specified.");
      return;
    } 
  
  if(color[0].size() < 4 || color[1].size() < 4)
    {
      ROS_INFO("[visualizeDetailedStates] RGBA must be specified for each color.");
      return;
    } 
  
  //if there are too many states, rviz will crash and burn when drawing
  if(states.size() > 50000)
    inc = 20;
  else if(states.size() > 5000)
    inc = 10;
  else if(states.size() > 500)
    inc = 2;
  else
    inc = 1;
    
  unsigned int mind = 0;
  for(unsigned int i = 0; i < states.size(); i=i+inc)
    {
      marker_array.markers.resize(marker_array.markers.size()+1);
      marker_array.markers[mind].header.frame_id = reference_frame_;
      marker_array.markers[mind].header.stamp = ros::Time::now();
      marker_array.markers[mind].ns = "expanded_states";
      marker_array.markers[mind].id = mind;
      marker_array.markers[mind].type = visualization_msgs::Marker::CUBE;
      marker_array.markers[mind].action =  visualization_msgs::Marker::ADD;
      marker_array.markers[mind].scale.x = size;
      marker_array.markers[mind].scale.y = size;
      marker_array.markers[mind].scale.z = size;
    
      for(unsigned int j = 0; j < 4; ++j)
	scaled_color[j] = color[0][j] - ((color[0][j] - color[1][j]) * (double(i)/double(states.size()/inc)));
      
      marker_array.markers[mind].color.r = scaled_color[0];
      marker_array.markers[mind].color.g = scaled_color[1];
      marker_array.markers[mind].color.b = scaled_color[2];
      marker_array.markers[mind].color.a = 1;
      marker_array.markers[mind].lifetime = ros::Duration(90.0);
    
      marker_array.markers[mind].pose.position.x = states[i][0];
      marker_array.markers[mind].pose.position.y = states[i][1];
      marker_array.markers[mind].pose.position.z = states[i][2];
    
      ++mind;
    } 
  
  ROS_DEBUG("[visualizeDetailedStates] published %d markers for %s states", (int)marker_array.markers.size(), name.c_str());
  marker_array_publisher_.publish(marker_array);
}
