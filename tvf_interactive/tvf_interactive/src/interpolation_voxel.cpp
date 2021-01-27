
#include <ros/ros.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init( argc, argv, "interpolation_voxel") ;
  ros::NodeHandle n;
  ros::Rate r( 1 );
  ros::Publisher marker_pub =
    n.advertise< visualization_msgs::Marker >( "interpolation_voxel_markers",
                                               1 );

  while ( ros::ok( ) )
  {
    visualization_msgs::Marker pt, s_intra, s_2intra, c_intra, c_2intra;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    pt.header.frame_id =       "/table_link";
    s_intra.header.frame_id =  "/table_link";
    s_2intra.header.frame_id = "/table_link";
    c_intra.header.frame_id = "/table_link";
    c_2intra.header.frame_id =  "/table_link";
    //
    pt.header.stamp =       ros::Time::now( );
    s_intra.header.stamp =  ros::Time::now( );
    s_2intra.header.stamp = ros::Time::now( );
    c_intra.header.stamp =  ros::Time::now( );
    c_2intra.header.stamp = ros::Time::now( );
    //
    pt.ns =       "interpolation_voxel";
    s_intra.ns =  "interpolation_voxel";
    s_2intra.ns = "interpolation_voxel";
    c_intra.ns =  "interpolation_voxel";
    c_2intra.ns = "interpolation_voxel";
    //
    pt.id =       0;
    s_intra.id =  1;
    s_2intra.id = 2;
    c_intra.id =  3;
    c_2intra.id = 4;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    pt.type = visualization_msgs::Marker::SPHERE;
    s_intra.type = visualization_msgs::Marker::SPHERE;
    s_2intra.type = visualization_msgs::Marker::SPHERE;
    c_intra.type = visualization_msgs::Marker::CUBE;
    c_2intra.type = visualization_msgs::Marker::CUBE;
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    pt.action = visualization_msgs::Marker::ADD;
    s_intra.action = visualization_msgs::Marker::ADD;
    s_2intra.action = visualization_msgs::Marker::ADD;
    c_intra.action = visualization_msgs::Marker::ADD;
    c_2intra.action = visualization_msgs::Marker::ADD;
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    pt.pose.position.x = 0.0;
    pt.pose.position.y = 0.0;
    pt.pose.position.z = 0.0;
    pt.pose.orientation.x = 0.0;
    pt.pose.orientation.y = 0.0;
    pt.pose.orientation.z = 0.0;
    pt.pose.orientation.w = 1.0;
    //
    s_intra.pose.position.x = 0.0;
    s_intra.pose.position.y = 0.0;
    s_intra.pose.position.z = 0.0;
    s_intra.pose.orientation.x = 0.0;
    s_intra.pose.orientation.y = 0.0;
    s_intra.pose.orientation.z = 0.0;
    s_intra.pose.orientation.w = 1.0;
    //
    s_2intra.pose.position.x = 0.0;
    s_2intra.pose.position.y = 0.0;
    s_2intra.pose.position.z = 0.0;
    s_2intra.pose.orientation.x = 0.0;
    s_2intra.pose.orientation.y = 0.0;
    s_2intra.pose.orientation.z = 0.0;
    s_2intra.pose.orientation.w = 1.0;
    //
    c_intra.pose.position.x = 0.0;
    c_intra.pose.position.y = 0.0;
    c_intra.pose.position.z = 0.0;
    c_intra.pose.orientation.x = 0.0;
    c_intra.pose.orientation.y = 0.0;
    c_intra.pose.orientation.z = 0.0;
    c_intra.pose.orientation.w = 1.0;
    //
    c_2intra.pose.position.x = 0.0;
    c_2intra.pose.position.y = 0.0;
    c_2intra.pose.position.z = 0.0;
    c_2intra.pose.orientation.x = 0.0;
    c_2intra.pose.orientation.y = 0.0;
    c_2intra.pose.orientation.z = 0.0;
    c_2intra.pose.orientation.w = 1.0;
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    pt.scale.x = 0.01;
    pt.scale.y = 0.01;
    pt.scale.z = 0.01;
    //
    s_intra.scale.x = 0.1;
    s_intra.scale.y = 0.1;
    s_intra.scale.z = 0.1;
    //
    s_2intra.scale.x = 0.2;
    s_2intra.scale.y = 0.2;
    s_2intra.scale.z = 0.2;
    //
    c_intra.scale.x = 0.1;
    c_intra.scale.y = 0.1;
    c_intra.scale.z = 0.1;
    //
    c_2intra.scale.x = 0.2;
    c_2intra.scale.y = 0.2;
    c_2intra.scale.z = 0.2;
    // Set the color -- be sure to set alpha to something non-zero!
    pt.color.r = 1.0f;
    pt.color.g = 1.0f;
    pt.color.b = 1.0f;
    pt.color.a = 1.0;
    //
    s_intra.color.r = 1.0f;
    s_intra.color.g = 0.0f;
    s_intra.color.b = 0.0f;
    s_intra.color.a = 0.8;
    //
    s_2intra.color.r = 0.0f;
    s_2intra.color.g = 0.0f;
    s_2intra.color.b = 1.0f;
    s_2intra.color.a = 0.6;
    //
    c_intra.color.r = 0.0f;
    c_intra.color.g = 1.0f;
    c_intra.color.b = 0.0f;
    c_intra.color.a = 0.4;
    //
    c_2intra.color.r = 0.0f;
    c_2intra.color.g = 0.0f;
    c_2intra.color.b = 0.0f;
    c_2intra.color.a = 0.2;
    //
    pt.lifetime = ros::Duration( );
    s_intra.lifetime = ros::Duration( );
    s_2intra.lifetime = ros::Duration( );
    c_intra.lifetime = ros::Duration( );
    c_2intra.lifetime = ros::Duration( );
    // Check for subscribers
    while ( marker_pub.getNumSubscribers( ) < 1 )
    {
      if ( !ros::ok( ) )
      {
        return 0;
      }
      ROS_WARN_ONCE( "Please create a subscriber to the marker" );
      sleep( 1 );
    }
    // Publish the marker
    marker_pub.publish( pt );
    sleep( 0.01 );
    marker_pub.publish( s_intra );
    sleep( 0.01 );
    marker_pub.publish( s_2intra );
    sleep( 0.01 );
    marker_pub.publish( c_intra );
    sleep( 0.01 );
    marker_pub.publish( c_2intra );

    r.sleep( );
  }
}