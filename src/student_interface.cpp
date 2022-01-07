#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>


#include <iostream>
#include <vector>
#include <chrono>

#include "rm/RoadMap.hpp"
#include "rm/maxClearance.hpp"
#include "rm/inflate.hpp"
#include "rm/geometry.hpp"



namespace student
{

  void loadImage(cv::Mat &img_out, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED");
  }

  void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED");
  }

  bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED");
  }

  void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                      const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
  {

    throw std::logic_error("STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED");
  }

  void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                          const cv::Mat &tvec, const std::vector<cv::Point3f> &object_points_plane,
                          const std::vector<cv::Point2f> &dest_image_points_plane,
                          cv::Mat &plane_transf, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED");
  }

  void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
              const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED");
  }

  bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED");
  }

  bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED");
  }

  bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list, const std::vector<Polygon> &gate_list, const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta, std::vector<Path> &path, const std::string &config_folder)
  {
    //throw std::logic_error("STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED");

    // TODO: Inflate obstacles. -- Implemented 
    // TODO: Construct maximum clearance graph. -- Implemented 
    // TODO: Build multilayer roadmap with finite set of orientations per node - connect only feasible dubins and save length of each edge.
    // TODO: Implement optimal path finding algorithm. -- Dijkstra 
    // TODO: Plan evader movements - random switch between gates at each node
    // TODO: Plan pursuer movements according to evader state (synchronously! Can not use information about future states of the evader, only know which node it is heading to next)
  
bool orientation = true; 
float offset = 2.0; // ---- change value accordingly 
bool debug_print = true; 



std::vector<Polygon> infObstacles = rm::inflate(obstacles, offset, orientation);

// --- RoadMap --- // --------------------------- TODO: include the gates list in the roadmap 
rm::RoadMap rm = rm::maxClearance(infObstacles, borders);

// ------------------------ // show the outcome // ------------------ // 
if(debug_print)
std::cout << "number of nodes in rm: " << rm.getNodeCount() << std::endl;

// ---------------------- Dubins connections ------------------------ // 
auto tic = std::chrono::high_resolution_clock::now();
auto n_connections = rm.build(8, 10.0f, infObstacles, infBorders);
auto toc = std::chrono::high_resolution_clock::now();

std::chrono::duration<float, std::milli> duration = toc - tic;

if(debug_print)
std::cout << "Generated " << n_connections << " dubins connections in " << duration.count() << " milliseconds." << std::endl;

    
  return true;
  }
}

