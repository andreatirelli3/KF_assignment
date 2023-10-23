#include <Eigen/Geometry>

namespace viewer
{

  struct BoxQ
  {
    Eigen::Vector3f bbox_transform;
    Eigen::Quaternionf bbox_quaternion;
    float cube_length;
    float cube_width;
    float cube_height;
  };

  struct Box
  {
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
  };

  struct Color
  {

    float r, g, b;

    Color(float setR, float setG, float setB)
        : r(setR), g(setG), b(setB)
    {
    }
  };

  enum class CameraAngle
  {
    XY,
    TopDown,
    Side,
    FPS
  };

}
