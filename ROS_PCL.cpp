
#include <stdio.h>
#include <ros/ros.h>
#include <iostream>
#include "math.h"
#include <boost/foreach.hpp>
// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>



#define radian      M_PI / 180
#define deg         180 / M_PI


ros::Publisher pub;



static inline bool convert (const sensor_msgs::PointCloud &input, sensor_msgs::PointCloud2 &output, int row[], int len)
{ 
  output.header = input.header;
  output.width  = 640*len;
  output.height = 1;
  output.fields.resize (3 + input.channels.size ());

  // Convert x/y/z to fields
  output.fields[0].name = "x"; output.fields[1].name = "y"; output.fields[2].name = "z";

  int offset = 0;

  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < output.fields.size (); ++d, offset += 4)
  {
    output.fields[d].offset = offset;
    output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
    output.fields[d].count  = 1;
  }

  output.point_step = offset; //16   //  output.data.size() = 10240 = 640*16
  output.row_step   = output.point_step * output.width;

  // Convert the remaining of the channels to fields
  for (size_t d = 0; d < input.channels.size (); ++d)
    output.fields[3 + d].name = input.channels[d].name;
  output.data.resize (640 * len * output.point_step);
  output.is_bigendian = false;  // @todo ?
  output.is_dense     = false;

// Copy the data points
  for(int i = 0 ; i < len ; ++i)
  {      
    for (size_t cp = 0 ; cp < 640 ; ++cp)
    {
        int wh = 640*row[i];  // input start point
        int pos = 640* i + cp; // output start point
        memcpy (&output.data[pos * output.point_step + output.fields[0].offset], &input.points[cp+wh].x, sizeof (float));
        memcpy (&output.data[pos * output.point_step + output.fields[1].offset], &input.points[cp+wh].y, sizeof (float));
        memcpy (&output.data[pos * output.point_step + output.fields[2].offset], &input.points[cp+wh].z, sizeof (float));

        for (size_t d = 0; d < input.channels.size (); ++d) // 1
        {
          if (input.channels[d].values.size() == 640*len)
          {
            memcpy (&output.data[pos * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[(cp+wh)], sizeof (float));
          }
        }
      }
  }
//    std::cout << "Pub OK" << std::endl;
  return (true);
}




void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.      
//      sensor_msgs::PointCloud2 input2;
      sensor_msgs::PointCloud2 output;
      sensor_msgs::PointCloud temp;

  //    voxel(*input, input2);

   //   noise_remove(*input, input2);

 //      int row[] = {0,100,200,300,400};


       //// 전체 출력 ////
      int row[480];
      for(int r = 0 ; r < 480 ; ++r )
         row[r] = r;

      int len = sizeof(row) / sizeof(row[0]);

     // sensor_msgs::convertPointCloud2ToPointCloud(input2, temp);
      sensor_msgs::convertPointCloud2ToPointCloud(*input, temp);
      convert(temp, output, row, len);


      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      // Conver the ROS msg to cloud
      pcl::fromROSMsg(output, *cloud);

      double minDistance=0.0;
      double min_angle_radx=0.0;
      double min_angle_rady=0.0;
      double xX=0.0,yY=0.0,zZ=0.0;
      int count=0;

      double theta = 0.0;     

      BOOST_FOREACH (const pcl::PointXYZRGB& pt, cloud->points)
      {
          if(atan2(pt.z, pt.y)*(deg)>85.00)
          {// atan2(z,y)= arctan(z/y) if z>0;
            // truncating points with less that 85 degree vertical angle
            // because the point formed could be ground.
              if(count==0)
              {
                  // initializing the first point read as minimum distance point
                  minDistance=hypot(pt.z, hypot(pt.x, pt.y));
                  min_angle_radx=atan2(pt.z,pt.x);
                  min_angle_rady=atan2(pt.z, pt.y);                  
                  xX=pt.x;  yY=pt.y;  zZ=pt.z;
                  theta = acos(zZ/ minDistance);
                  count++;

              }
             else if(hypot(pt.z, hypot(pt.x, pt.y))<minDistance) // sqrt(x*x + y*y)
              {
                  // keep updating the minimum Distant point
                  minDistance=hypot(pt.z, hypot(pt.x, pt.y));
                  min_angle_radx=atan2(pt.z,pt.x);
                  min_angle_rady=atan2(pt.z, pt.y);                  
                  xX=pt.x;  yY=pt.y;  zZ=pt.z;
                  theta = acos(zZ/minDistance);
              }
              else{ continue; }
            }          
      }

      std::cout << "-------------------------------------------------" << std::endl;
      std::cout << "Distance = " << minDistance << "\n";
      std::cout << "Angle in Degree X axis = " << min_angle_radx*(deg) << "\n";
      std::cout << "Angle in Degree Y axis = " << min_angle_rady*(deg) << "\n";
      std::cout << "theta = " << theta*(deg) << "\n";
      std::cout << "atan = " << atan2(zZ, yY)*(deg) << "\n";
      std::cout << "point X coordinate = " << xX << "\n";
      std::cout << "point Y coordinate = " << yY << "\n";
      std::cout << "point Z coordinate = " << zZ << "\n" << std::endl;
      //sleep(1);use sleep if you want to delay loop.



/*
      int c = cloud->points.size()/len/2 - 320;

      float x = cloud->points[c].x;
      float y = cloud->points[c].y;
      float z = cloud->points[c].z;
      float r2 = pow(x,2) + pow(y,2) + pow(z,2);
      float r = sqrt(r2);
      float theta = acos(z/r) * radian ;
      float pi = atan(y/z);

*/


  //  float theta = M_PI/8;


      Eigen::Affine3f transform = Eigen::Affine3f::Identity();


       // The same rotation matrix as before; theta radians around X axis
       transform.rotate (Eigen::AngleAxisf (0, Eigen::Vector3f::UnitX()));

       // Define a translation of 0.0 meters on the x axis.
       transform.translation() << 0.0, 0.0, 0.0;

       /*
       // Print the transformation
       printf ("\n--------------------------------------\n");
       std::cout << "deg = " << theta * deg << ", r = " << r << std::endl;
       std::cout << x << ",  " << y << ",  " << z << std::endl;
       std::cout << transform.matrix() << std::endl;
*/

       // Executing the transformation
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
       pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
       pcl::toROSMsg(*transformed_cloud, output);

      // Conver the cloud to ROS msg
  //     pcl::toROSMsg(*cloud, output);

    // Publish the data.
      pub.publish (output);      
}

//----------------------------------------------------------------------------------------------//

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}


