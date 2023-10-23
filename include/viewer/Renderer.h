
//
// The original author of the rendering code is Aaron Brown (https://github.com/awbrown90).
// His code has been slightly modified to make it more structured.
//

#ifndef RENDERER_HPP
#define RENDERER_HPP

#include <iostream>
#include <vector>
#include <string>

#include <pcl/visualization/pcl_visualizer.h>

#include "viewer/viewer_utils.hpp"

namespace viewer
{

  static bool lidarActivated;

  class Renderer
  {
  private:
    pcl::visualization::PCLVisualizer::Ptr viewer_;
    unsigned long long rays_counter_;

  public:
    static void setLidarStatus();

    static bool getLidarStatus();

    Renderer();

    static void keyboardCallback(const pcl::visualization::KeyboardEvent &event);

    void removeShape(int id);

    void addCircle(float centroid_x, float centroid_y, int id);

    void clearRays();

    void clearViewer();

    void addText(float centroid_x, float centroid_y, int id);

    void renderPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                          const std::string &name,
                          const Color &color = Color(1, 1, 1));

    void renderPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                          const std::string &name,
                          const Color &color = Color(-1, -1, -1));

    void renderBox(const Box &box, int id, const Color &color = Color(1, 0, 0), float opacity = 1.0);

    void renderBox(const BoxQ &box, int id, const Color &color = Color(1, 0, 0), float opacity = 1.0);

    void initCamera(CameraAngle view_angle);

    bool wasViewerStopped() const;

    void spinViewerOnce() const;
  };
}

#endif // RENDERER_HPP
