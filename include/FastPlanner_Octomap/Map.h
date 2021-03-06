/** 
 * Map header file
 * Store the parameters of the map
**/
#pragma once

#include"utils.h"

namespace Map3D
{
    class OctoMapEDT
    {
        public:

            //// data members ////
            octomap::AbstractOcTree *new_tree = NULL; // reads the octomap when available in callback
            octomap::OcTree *tree = new octomap::OcTree(0.05); // convert to OcTree for EDT calculation            
            octomap::point3d min, max;  // min, max of the map
            octomap::point3d start, end; // start, end of the fixed window EDT
            octomap::point3d mapStart; // location where mapping starts or reset
            float mapUpdateThreshold;  // minimum distance the drone should travel w.r.to previous mapUpdate point for map to update again
            DynamicEDTOctomap* costMap;    // pointer to the EDT of the octomap
            float xRange, yRange, zRange; // range in x,y,z dimensions (X->forward, y->sideways, z->vertical)
            bool isOctomapUpdated;       // set true when map is updated
            
            //// member functions /////
            OctoMapEDT();
            bool ifUpdateMap(Eigen::Vector3d pt1);
            void setStartPosition(Eigen::Vector3d pt);
            void setMapRange(Eigen::Vector3d pt);
            void setMinMax();
            void erase();
            void getCostMapMarker(visualization_msgs::MarkerArray m, DynamicEDTOctomap *ptr, ros::Publisher pub);
        
        private:
            Eigen::Vector3d convertToEigenPt(octomap::point3d pt); // convert octomap::point3d to Eigen::Vector3d
            octomap::point3d convertToOctomapPt(Eigen::Vector3d pt); // convert Eigen::Vector3d to octomap::point3d

    };
}


//////////////////////////////////////////////////////////////////////////////////////
/** set start and end of Map to (0,0,0) **/
Map3D::OctoMapEDT::OctoMapEDT()
{
    std::cout<<"_____Mapping initialized ..."<<std::endl;

    std::cout<<"\n";

    xRange =  8.0; // forward (0<x<15)
    yRange = 6.0;  // sideways(-3<y<3)
    zRange = 6.0;  // vertical (-3<z<3)

    mapUpdateThreshold = 1.0;

    std::cout<<"_______Setting the EDT range ... "<<"("<<xRange<<","<<yRange<<","<<zRange<<")"<<std::endl;
    std::cout<<"\n";

}


///////////////////////////////////////////////////////////////////////////////////////
/** delete the trees to save memory **/
void Map3D::OctoMapEDT::erase()
{
    delete tree;
    delete new_tree;
}


/////////////////////////////////////////////////////////////////////////////////////
/** get minimum and maximum in an octomap **/
void Map3D::OctoMapEDT::setMinMax()
{
    double x,y,z;
    tree->getMetricMax(x,y,z);

    max.x() = x;
    max.y() = y;
    max.z() = z; 

    tree->getMetricMin(x,y,z);
    min.x() = x;
    min.y() = y;
    min.z() = z;
}


/////////////////////////////////////////////////////////////////////////////////////
/** Check if the drone has travelled enough distance to update the map again **/
bool Map3D::OctoMapEDT::ifUpdateMap(Eigen::Vector3d pt1)
{
    Eigen::Vector3d pt = convertToEigenPt(mapStart);
    Eigen::Vector3d diff = pt1 - pt;
    
    if(diff.norm()>mapUpdateThreshold)
    {
        std::cout<<">>>>>>>>>>>> Map Updated from location "<<pt1.transpose()<< "to "<<pt.transpose()<<std::endl;
        std::cout<<"\n";
        return true;
    }
    else
    {
        return false;
    }
    
}


/////////////////////////////////////////////////////////////////////////////////////
/** set mapStart to the point where mapping starts/re-initializes **/
void Map3D::OctoMapEDT::setStartPosition(Eigen::Vector3d pt)
{
    mapStart = convertToOctomapPt(pt);
}


////////////////////////////////////////////////////////////////////////////////////
/**
 * set the start and end points of the map using the current pose, 
 * max ranges along each axis and min/max of octomap 
 **/

void Map3D::OctoMapEDT::setMapRange(Eigen::Vector3d pt)
{
    octomap::point3d p = convertToOctomapPt(pt);

    std::cout<<"Octomap point "<<p<<std::endl;
    
   /** set start coordinates **/

    // X/Forward measurement is currPose + xRange;
    start.x() = p.x() - xRange;

    // For Y and Z dimensions, check if map of desired size (8,6,6) is possible or not. If not, then set map size to whatever is possible
    if(p.y() - yRange/2.0 > min.y())
    {
        start.y() = p.y() - yRange/2.0;
    }
    else
    {
        start.y() = min.y();
    }

    if(p.z() - zRange/2.0 > min.z())
    {
        start.z() = p.z() - zRange/2.0;
    }
    else
    {
        start.z() = min.z();
    }    

    start.y() = p.y() - xRange;//yRange/2.0;
    start.z() = p.z() - xRange;//zRange/2.0;

   /** set end coordinates **/
   end.x() = p.x() + xRange;

   if(p.y() + yRange/2.0 > max.y())
    {
        end.y() = max.y();
    }
   else
    {
        end.y() = p.y() + yRange/2.0;
    }

   if(p.z() + zRange/2.0 > max.z())
    {
        end.z() = max.z();
    }
   else
    {
        end.z() = p.z() + zRange/2.0;
    }   

    end.y() = p.y() + xRange;
    end.z() = p.z() + xRange;

    std::cout<<"Map from "<<start<<" to "<<end<<std::endl;

}


//////////////////////////////////////////////////////////////////////////////////////////
/* Get the marker array for visualizing the cost map **/
/*
void Map3D::OctoMapEDT::getCostMapMarker(visualization_msgs::MarkerArray m, DynamicEDTOctomap *ptr, ros::Publisher pub)
{
    uint32_t shape = visualization_msgs::Marker::CUBE;
    int count = 0;
    double z = start.z() + end.z();
    z = z/2.0;

    for(octomap::OcTree::leaf_iterator it = start; it != end; ++it)
    {
        std::cout<<"Hello there!"<<std::endl;
    }

    for(double x = start.x(); x<end.x(); x++)
    {
        for(double y = start.y(); y<end.y(); y++)
            {
                        if(!ros::ok())
                        {
                            break;
                        }
                        // select the color using max Distance
                        visualization_msgs::Marker marker;

                        marker.header.frame_id = "map";
                        marker.header.stamp = ros::Time::now();
                        marker.ns = x+y+z;
                        marker.id = 0;
                        marker.type = shape;

                        // set the marker actions. Options are add and delete
                        marker.action = visualization_msgs::Marker::ADD;


                        // set the pose of the marker. This is a full 6DOF pose relative to the frame/time specified in the header
                        marker.pose.position.x = x;
                        marker.pose.position.y = y;
                        marker.pose.position.z = z;

                        marker.pose.orientation.x = 0.0;
                        marker.pose.orientation.y = 0.0;
                        marker.pose.orientation.z = 0.0;
                        marker.pose.orientation.w = 1.0;

                        // set the scale of the marker
                        marker.scale.x = 0.75;
                        marker.scale.y = 0.75;
                        marker.scale.z = 0.75;

                        // get the distance at each index in order to set the color for each voxel
                        octomap::point3d pt(x,y,z);
                        float dist = ptr->getDistance(pt);
                        
                        //std::cout<<dist<<std::endl;

                        // Set the color -- be sure to set alpha to something non-zero!
                        marker.color.r = 1 - (dist/5.0)*(dist/5.0);
                        marker.color.g = 0.0;//1.0 - dist/5.0;
                        marker.color.b = 0.0;//1.0 - dist/5.0;
                        marker.color.a = 1.0;
                    
                        marker.lifetime = ros::Duration();
                        m.markers.push_back(marker);
            }
    }

    pub.publish(m);

}
*/

//////////////////////////////////////////////////////////////////////////////////////////
/** publish the costmap obtained from the octree **/
void Map3D::OctoMapEDT::getCostMapMarker(visualization_msgs::MarkerArray m, DynamicEDTOctomap* ptr, ros::Publisher pub)
{
    unsigned char maxDepth = 16;
    uint32_t shape = visualization_msgs::Marker::CUBE;
    int count  = 0;

    // set a bounding box using the start and end variables
    for(octomap::OcTree::leaf_bbx_iterator it = tree->begin_leafs_bbx(start, end, maxDepth), bbx_end = tree->end_leafs_bbx(); it != bbx_end; ++it)
    {
        std::cout<<it.getCoordinate()<<std::endl;
        octomap::point3d pt = it.getCoordinate();

        if(!ros::ok())
        {
            break;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = count;//pt.x() + pt.y() + pt.z();
        marker.id = count;
        marker.type = shape;

        count++;

        // set the marker action. Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // set the pose of the marker with respect to the global frame of reference
        marker.pose.position.x = pt.x();
        marker.pose.position.y = pt.y();
        marker.pose.position.z = pt.z();

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        // set the scale of the marker
        marker.scale.x = 0.25;
        marker.scale.y = 0.25;
        marker.scale.z = 0.25;

        // set the color of the cell based on the distance from the obstacle
        float dist = ptr->getDistance(pt);
                        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 1 - dist/5.0*dist/5.0;
        marker.color.g = dist/5.0*dist/5.0;//dist/5.0;
        marker.color.b = 0.0;//1.0 - dist/5.0;
        marker.color.a = 0.1;
                    
        marker.lifetime = ros::Duration();
        m.markers.push_back(marker);
        
    }
        pub.publish(m);
}

//////////////////////////////////////////////////////////////////////////////////////////
/** convert to octomap::point3d from Eigen::Vector3d **/
octomap::point3d Map3D::OctoMapEDT::convertToOctomapPt(Eigen::Vector3d pt)
{
    octomap::point3d p(pt(0), pt(1), pt(2));
    return p;
}


////////////////////////////////////////////////////////////////////////////////////////
/** convert to Eigen::Vector3d from octomap::point3d **/
Eigen::Vector3d Map3D::OctoMapEDT::convertToEigenPt(octomap::point3d pt)
{
    Eigen::Vector3d p(pt.x(), pt.y(), pt.z());
    return p;
}
