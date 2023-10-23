
//
// The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
// His code has been slightly modified to make it more structured.
//

#include "viewer/Renderer.h"

namespace viewer
{

  void Renderer::addCircle(float centroid_x, float centroid_y, int id)
  {

    pcl::ModelCoefficients c_coeff;
    c_coeff.values.resize(3);
    c_coeff.values[0] = centroid_x;
    c_coeff.values[1] = centroid_y;
    c_coeff.values[2] = 0.4; // radius

    viewer_->addCircle(c_coeff, "c_" + std::to_string(id), 0);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 1000, "c_" + std::to_string(id));
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "c_" + std::to_string(id));
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 10000.0, "c_" + std::to_string(id));
  }

  void Renderer::addText(float centroid_x, float centroid_y, int id)
  {

    viewer_->addText3D(std::to_string(id), pcl::PointXYZ(centroid_x, centroid_y, 0), 0.3, 255, 255, 255, "", 0);
  }

  void Renderer::removeShape(int id)
  {
    viewer_->removeShape("c_" + std::to_string(id), 0);
  }

  void Renderer::keyboardCallback(const pcl::visualization::KeyboardEvent &event)
  {
    if (event.getKeySym() == "v" && event.keyUp())
    {
      setLidarStatus();
    }
  }

  void Renderer::setLidarStatus()
  {
    lidarActivated ^= true;
  }

  bool Renderer::getLidarStatus()
  {
    return lidarActivated;
  }

  void Renderer::clearRays()
  {
    while (rays_counter_-- > 0)
    {
      viewer_->removeShape("ray" + std::to_string(rays_counter_));
    }
  }

  void Renderer::renderPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const std::string &name, const Color &color)
  {

    viewer_->addPointCloud<pcl::PointXYZ>(cloud, name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
  }

  void Renderer::renderPointCloud(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, const std::string &name, const Color &color)
  {

    if (color.r == -1)
    {
      // Select color based off of cloud intensity
      pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
      viewer_->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
    }
    else
    {
      // Select color based off input value
      viewer_->addPointCloud<pcl::PointXYZI>(cloud, name);
      viewer_->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    }

    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, name);
  }

  // Draw wire frame box with filled transparent color
  void Renderer::renderBox(const Box &box, const int id, const Color &color, float opacity)
  {
    if (opacity > 1.0)
      opacity = 1.0;
    if (opacity < 0.0)
      opacity = 0.0;

    std::string cube = "box" + std::to_string(id);

    viewer_->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);

    viewer_->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max,
                     color.r, color.g, color.b, cubeFill);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);

    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
  }

  void Renderer::renderBox(const BoxQ &box, const int id, const Color &color, float opacity)
  {
    if (opacity > 1.0)
      opacity = 1.0;
    if (opacity < 0.0)
      opacity = 0.0;

    std::string cube = "box" + std::to_string(id);
    viewer_->addCube(box.bbox_transform, box.bbox_quaternion, box.cube_length, box.cube_width, box.cube_height, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cube);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity, cube);

    std::string cubeFill = "boxFill" + std::to_string(id);
    viewer_->addCube(box.bbox_transform, box.bbox_quaternion,
                     box.cube_length, box.cube_width, box.cube_height, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, cubeFill);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opacity * 0.3, cubeFill);
  }

  void Renderer::initCamera(CameraAngle view_angle)
  {

    viewer_->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer_->initCameraParameters();

    // distance away in meters
    int distance = 10;

    switch (view_angle)
    {
    case CameraAngle::XY:
    {
      viewer_->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
      break;
    }
    case CameraAngle::TopDown:
    {
      viewer_->setCameraPosition(0, 0, distance, 1, 0, 1);
      break;
    }
    case CameraAngle::Side:
    {
      viewer_->setCameraPosition(0, -distance, 0, 0, 0, 1);
      break;
    }
    case CameraAngle::FPS:
    {
      viewer_->setCameraPosition(-10, 0, 0, 0, 0, 1);
      break;
    }
    default:
    {
      throw std::logic_error("unknown CameraAngle");
    }
    }

    if (view_angle != CameraAngle::FPS)
    {
      viewer_->addCoordinateSystem(1.0);
    }
    viewer_->registerKeyboardCallback(keyboardCallback);
    lidarActivated = true;
  }

  void Renderer::clearViewer()
  {
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
  }

  bool Renderer::wasViewerStopped() const
  {
    return viewer_->wasStopped();
  }

  Renderer::Renderer() : viewer_{new pcl::visualization::PCLVisualizer("3D Viewer")}, rays_counter_{0}
  {
  }

  void Renderer::spinViewerOnce() const
  {
    viewer_->spinOnce();
  }
}
