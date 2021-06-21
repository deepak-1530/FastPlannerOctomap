// publish query point for which edt is needed on tje topic "/query_point_topic"
// calculate EDT and publish the distance on the topic "/query_point_distance"

#include"ros/ros.h"
#include"geometry_msgs/PoseStamped.h"
#include"std_msgs/Float64.h"
#include<iostream>

ros::Subscriber edtDistSub;
ros::Publisher  edtQueryPub;

void edt_query_cb(std_msgs::Float64 msg)
{
    std::cout<<msg.data<<std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "EDT_query");
    ros::NodeHandle nh;

    edtQueryPub = nh.advertise<geometry_msgs::PoseStamped>("/query_point_topic", 1);
    edtDistSub = nh.subscribe<std_msgs::Float64>("/query_point_distance", 1, edt_query_cb);

    while(ros::ok())
    {
        if (!ros::ok())
        {
            break;
        }
        geometry_msgs::PoseStamped queryPt;
        std::cout<<"Enter the query point ";
        std::cin>>queryPt.pose.position.x>>queryPt.pose.position.y>>queryPt.pose.position.z;

        edtQueryPub.publish(queryPt);
        ros::spinOnce();

        
    }

    return 0;
    
}