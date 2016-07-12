/* \author Kalin Gochev */

#ifndef __NAVVIZ_H_
#define __NAVVIZ_H_

#include <string>
#include <fstream>
#include <stdlib.h>
#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <boost/lexical_cast.hpp>

#define TWO_PI 6.2831853


  
class NavVisualizer
{
  public:
    
    NavVisualizer(std::string robot_name, double robotW, double robotH, double offX, double offY, double offZ);

    ~NavVisualizer();
    
    void advertiseTransform();

    /* \brief set reference frame of visualizations */
    void setReferenceFrame(std::string &frame);

    /* \brief visualize configuration */
    void visualizeConfiguration(double color_num, const std::vector<double> &coords, std::string name);

    /* \brief visualize the arm of the pr2 in specified color (color: 1-360) */
    void visualizeMesh(double color_num, std::vector<geometry_msgs::PoseStamped> &poses, std::string name);
    void visualizeMeshes(const std::vector<double> &colors, std::vector<geometry_msgs::PoseStamped> &poses, std::string name);

    /* \brief visualize cuboids */
    void visualizeSimpleObstacles(const std::vector<std::vector<double> > &obstacles, std::string *objName, int color, double trans);
    void visualizeComplexObstacles(const std::vector<std::vector<double> > &obstacles, std::string *objName, int color, double trans);
   
    void visualizePath(std::vector<std::vector<double> > &dpath, int scolor, int ecolor, std::string ns);
    inline void visualizePath(std::vector<std::vector<double> > &dpath, int scolor, int ecolor){
    	visualizePath(dpath, scolor, ecolor, "");
    }

    /* \brief display a throttled set of arm configurations in a trajectory
     * by default throttle = 5 */
    void visualizeConfigurations(std::string name, double scolor, double ecolor, const std::vector<std::vector<double> > &traj, int throttle);

    /* \brief display a list of states (xyz coordinates) (intended for use with sbpl) */
    void visualizeBasicStates(const std::vector<std::vector<double> > &states, const std::vector<double> &color, std::string name, double size);
    
    /* \brief display a list of states (xyz coordinates with rpy arrows) (intended for use with sbpl) */
    void visualizeDetailedStates(const std::vector<std::vector<double> > &states, const std::vector<std::vector<double> >&color, std::string name, double size);

    /* \brief display a sphere */
    void visualizeSphere(std::vector<double> pose, int color, std::string text, double radius);
    void visualizeCircle(std::vector<double> pose, int color, std::string text, double radius);
    
    void visualizeCylinder(std::vector<double> p1, std::vector<double> p2, int color, std::string text, double radius);
    
    /* \brief display a list of spheres of the same radius and color */
    void visualizeSpheres(const std::vector<std::vector<double> > &pose, int color, std::string text, double radius);
    void visualizePoints(const std::vector<std::vector<double> > &pose, int color, std::string text);
    void visualizePoints(const std::vector<std::vector<double> > &pose, const std::vector<double> &colors, std::string text);
    void visualizeArrow(const std::vector<double> &begin, const std::vector<double> &ori, std::string *objName);
    void visualizeLine(std::vector<double> p1, std::vector<double> p2, int color, std::string name);

    /* DOESN'T WORK */
    //void clearAllVisualizations();

  private:

    tf::TransformBroadcaster br;
    std::vector<tf::TransformBroadcaster> mytransforms;

    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Publisher marker_array_publisher_;
    ros::Publisher marker_publisher_;
    ros::Publisher display_trajectory_publisher_;

    std::string reference_frame_;

    int num_joints_;

    visualization_msgs::MarkerArray marker_array_;
    visualization_msgs::Marker marker_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<std::string> meshes_;

    std::string robot_name_;

    std::vector<double> start_config_;
    std::vector<double> goal_pose_;
    std::vector<std::vector<double> > cubes_;
    double position_tolerance_;
    double orientation_tolerance_;
    
    double offsetX;
    double offsetY;
    double offsetZ;
    double robot_width_;
    double robot_height_;
};

#endif
