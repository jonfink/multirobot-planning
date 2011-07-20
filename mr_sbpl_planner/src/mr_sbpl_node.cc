#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <pthread.h>

#include "ros/ros.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>

#include <iostream>
#include <sstream>
#include <string>
#include <set>
using namespace std;

#include <tf/transform_listener.h>

tf::TransformListener *listener;
string base_frame; // e.g. /scarab1/base
string root_frame; // e.g. /map

#include "CentralizedSBPLPlanner.h"

CentralizedSBPLPlanner *sbpl;
ros::Publisher simple_planning_pub;
double time_budget;
bool waiting_for_plan;

void GetPosition(double &x, double &y, double &th)
{
  tf::StampedTransform transform;
  string frame_id = root_frame;
  string child_frame_id = base_frame;
  try
    {
      listener->lookupTransform(frame_id, child_frame_id,
                               ros::Time(0), transform);
    }
  catch(tf::TransformException e)
    {
      ROS_ERROR("%s: transform lookup from %s to %s failed", ros::this_node::getName().c_str(),
                frame_id.c_str(), child_frame_id.c_str());
      transform.setIdentity();
    }

  x = transform.getOrigin().x();
  y = transform.getOrigin().y();
  //double z = transform.getOrigin().z();

  geometry_msgs::TransformStamped transform_msg;
  tf::transformStampedTFToMsg(transform, transform_msg);
  th = tf::getYaw(transform_msg.transform.rotation);
}

void PublishCurrentPlan(list<map<int, pair<double, double> > > &simple_path)
{
  if(simple_path.size() > 0) {

    geometry_msgs::PoseArray pose_array_msg;

    pose_array_msg.poses.resize(simple_path.size());

    double z = 0.01;

    //printf("Publishing plan: ");

    int k=0;
    for(list<map<int, pair<double, double> > >::iterator wp=simple_path.begin();
        wp != simple_path.end(); ++wp) {

      for(map<int, pair<double,double> >::iterator i=wp->begin();
          i != wp->end(); ++i) {
        pose_array_msg.poses[k].position.x = i->second.first;
        pose_array_msg.poses[k].position.y = i->second.second;
        pose_array_msg.poses[k].position.z = z;
        //printf("(%d, %2.2f, %2.2f) ", k, i->second.first, i->second.second);
      }
      ++k;

    }
    //printf("\n");
    simple_planning_pub.publish(pose_array_msg);
  }
}

void OnGoal(const geometry_msgs::Vector3::ConstPtr &_msg)
{
  map<int, pair<double, double> > simple_goal;
  map<int, pair<double,double> > current_node_state;

  double x, y, th;
  GetPosition(x, y, th);

  simple_goal.insert(make_pair(0, make_pair(_msg->x, _msg->y)));
  current_node_state.insert(make_pair(0, make_pair(x, y)));

  sbpl->SetGoal(1, simple_goal);
  sbpl->PlanConfiguration(current_node_state, time_budget);

  waiting_for_plan = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_sbpl_planner");

  listener = new tf::TransformListener;

  ROSCONSOLE_AUTOINIT;
  log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
  ros::console::notifyLoggerLevelsChanged();

  ros::NodeHandle n("~");

  double freq;
  n.param("freq", freq, 10.0);
  n.param("time_budget", time_budget, 5.0);

  // Should be able to automatically set this based on tf_prefix param
  n.param("base_frame", base_frame, string("/scarab1/base"));
  n.param("root_frame", root_frame, string("/map"));

  sbpl = new CentralizedSBPLPlanner(&n);

  ros::Subscriber goal_sub = n.subscribe<geometry_msgs::Vector3>("goal", 1, &OnGoal);

  simple_planning_pub = n.advertise<geometry_msgs::PoseArray>("simple_planning_tree", 1, true);

  list<map<int, pair<double, double> > > simple_path;

  while(n.ok()) {
    ros::spinOnce();

    if(waiting_for_plan && !sbpl->IsPlanning()) {
      // Print/publish path
      if(sbpl->GetPath(simple_path, false)) {
        waiting_for_plan = false;
        PublishCurrentPlan(simple_path);
        // int k=0;
        // for(list<map<int, pair<double, double> > >::iterator iter = simple_path.begin();
        //     iter != simple_path.end(); ++iter) {
        //   printf("\t %d: %2.2f %2.2f\n", k++, (*iter)[0].first, (*iter)[0].second);
        // }
      }
    }
  }

  delete listener;

  return 0;
}

