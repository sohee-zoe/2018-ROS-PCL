#include <ros/ros.h>
#include <iostream>
#include "math.h"

// PCL specific includes
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_spherical.h>

#define radian M_PI / 180
#define deg 180/M_PI


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







//----------------------------------------------------------------------------------------------//

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Create a container for the data.
      sensor_msgs::PointCloud2 output;
      sensor_msgs::PointCloud temp;

      //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());



 //      int row[] = {0,100,200,300,400};


       //// 전체 출력 ////
      int row[480];
      for(int r = 0 ; r < 480 ; ++r )
         row[r] = r;


      int len = sizeof(row) / sizeof(row[0]);

      sensor_msgs::convertPointCloud2ToPointCloud(*input, temp);
      convert(temp, output, row, len);

      // Conver the ROS msg to cloud
      pcl::fromROSMsg(output, *cloud);
/*
      for (size_t i = 0; i < cloud->points.size (); ++i)
      {
          float x = cloud->points[i].x;
          float y = cloud->points[i].y;
          float z = cloud->points[i].z;
          float r2 = pow(x,2) + pow(y,2) + pow(z,2);
          float r = sqrt(r2);
          float theta = acos(z/r) * radian * (-1);
          float pi = atan(y/z);
      }*/


      float x = 0;
      float y = cloud->points[340].y;
      float z = cloud->points[340].z;
      float r2 = pow(x,2) + pow(y,2) + pow(z,2);
      float r = sqrt(r2);
      float theta = acos(z/r) * radian;
      float phi = atan(y/z);

      printf ("\n--------------------------------------\n");
      std::cout << x << ",  " << y << ",  " << z << std::endl
                << r << std::endl;

      float theta_m = 60*radian;

      Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    //  float theta = M_PI/8;


       // Define a translation of 0.0 meters on the x axis.
        transform.translation() << 0.0, 0.0, 0.0;

       // The same rotation matrix as before; theta radians around X axis
       transform.rotate (Eigen::AngleAxisf (theta_m, Eigen::Vector3f::UnitX()));


       // Print the transformation
       printf ("\nMethod #2: using an Affine3f\n");
       std::cout << theta * deg << std::endl;
       std::cout << x << ",  " << y << ",  " << z << std::endl;
       std::cout << transform.matrix() << std::endl;

       // Executing the transformation
       pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
       // You can either apply transform_1 or transform_2; they are the same
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



/*
//      Reminder: how transformation matrices work :
//
//                |-------> This column is the translation
//         | 1 0 0 x |  \
//         | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
//         | 0 0 1 z |  /
//         | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

//         METHOD #1: Using a Matrix4f
//         This is the "manual" method, perfect to understand but error prone !

       float theta = M_PI/4;

       Eigen::Matrix4f transform1 = Eigen::Matrix4f::Identity();

       // x축 중심 회전
       transform1 (0,0) = 1;
       transform1 (0,1) = 0;
       transform1 (0,2) = 0;
       transform1 (1,0) = 0;
       transform1 (1,1) = cos (theta);
       transform1 (1,2) = -sin (theta);
       transform1 (2,0) = 0;
       transform1 (2,1) = sin (theta);
       transform1 (2,2) = cos (theta);
      //    (row, column)

      printf ("Method #1: using a Matrix4f\n");
      std::cout << transform1 << std::endl;

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
      pcl::transformPointCloud (*cloud, *transformed_cloud, transform1);
      pcl::toROSMsg(*transformed_cloud, output);

*/



/*
//  METHOD #2: Using a Affine3f
//   This method is easier and less error prone


Eigen::Affine3f transform = Eigen::Affine3f::Identity();
float theta = M_PI/8;


 // Define a translation of 0.0 meters on the x axis.
//  transform.translation() << 0.0, 0.0, 0.0;

 // The same rotation matrix as before; theta radians around X axis
 transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));


 // Print the transformation
 printf ("\nMethod #2: using an Affine3f\n");
 std::cout << transform.matrix() << std::endl;

 // Executing the transformation
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());
 // You can either apply transform_1 or transform_2; they are the same
 pcl::transformPointCloud (*cloud, *transformed_cloud, transform);
 pcl::toROSMsg(*transformed_cloud, output);
 */


// http://docs.ros.org/api/pcl_ros/html/namespacepcl__ros.html

// https://www.slideshare.net/WoohyunKim16/3d-85975448
// http://egloos.zum.com/EireneHue/v/982268
// https://docs.microsoft.com/ko-kr/xamarin/xamarin-forms/user-interface/graphics/skiasharp/transforms/3d-rotation

// ??
// https://stackoverflow.com/questions/45570286/point-cloud-projection-to-2d
// http://pointclouds.org/documentation/tutorials/range_image_creation.php


/*  void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in,
                                     pcl::PointCloud< PointT > &  cloud_out,
                                     const Eigen::Matrix4f &  transform  )
     */
