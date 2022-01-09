#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <iostream>
#include <chrono>

#include "rm/RoadMap.hpp"
#include "rm/maxClearance.hpp"
#include "rm/inflate.hpp"
#include "nav/dijkstra.hpp"
#include "dubins/dubins.hpp"

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

	bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
				  const std::vector<Polygon> &gate_list,
				  const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
				  std::vector<Path> &path, const std::string &config_folder)
	{
		// TODO: Inflate obstacles. -- Implemented
		// TODO: Construct maximum clearance graph. -- Implemented
		// TODO: Build multilayer roadmap with finite set of orientations per node - connect only feasible dubins and save length of each edge.
		// TODO: Implement optimal path finding algorithm. -- Dijkstra
		// TODO: Plan evader movements - random switch between gates at each node
		// TODO: Plan pursuer movements according to evader state (synchronously! Can not use information about future states of the evader, only know which node it is heading to next)

		float robot_size = 0.2f;				// How can we get this from the simulator??
		int n_poses = 8;						// Number of poses per node
		float kmax = 3.0 / robot_size;			// Maximum curvature of Dubins paths
		float bypass_min = 1 / kmax;			// Minimum length of edges for making bypass connections
		float bypass_min_keep = bypass_min / 2; // Minimum length of edges for them to be kept in the final graph
		int k = 5;								// Robot free movement parameter
		float step = M_PI / 16 / kmax;			// Discretization step

		// Inflate obstacles and borders
		std::cout << "Inflating obstacles and borders..." << std::endl;
		auto tic = std::chrono::high_resolution_clock::now();

		auto infObstacles = rm::inflate(obstacle_list, robot_size, true);
		auto infBorders = rm::inflate(std::vector<Polygon>{borders}, -robot_size, false).back();

		auto toc = std::chrono::high_resolution_clock::now();
		std::chrono::duration<float, std::milli> duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Setup RoadMap by maximum clearance graph
		std::cout << "Setting up maximum clearance graph..." << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		rm::RoadMap rm = rm::maxClearance(infObstacles, infBorders);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Create bypass edges
		std::cout << "Creating bypass edges...[1/2]" << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		rm.bypass(bypass_min_keep, true);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		std::cout << "Creating bypass edges...[2/2]" << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		rm.bypass(bypass_min, false);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Build RoadMap
		std::cout << "Building roadmap..." << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		rm.build(n_poses, kmax, infObstacles, infBorders);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Add initial position
		std::cout << "Adding start pose..." << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		auto &source = rm.addStartPose(Point(x[0], y[0]), theta[0], k, kmax, infObstacles, infBorders);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Add gate position
		std::cout << "Adding goal pose..." << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		float gate_x = 0.0f;
		float gate_y = 0.0f;
		for (auto &p : gate_list[0])
		{
			gate_x += p.x;
			gate_y += p.y;
		}
		gate_x /= gate_list[0].size();
		gate_y /= gate_list[0].size();
		auto &goal = rm.addGoalPose(Point(gate_x, gate_y), M_PI_2, k, kmax, infObstacles, borders);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Get shortest path
		std::cout << "Running Dijkstra algorithm..." << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		auto nav_list = nav::dijkstraShortestPath(source, goal);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;

		// Discretize path
		std::cout << "Discretizing path..." << std::endl;
		tic = std::chrono::high_resolution_clock::now();

		float offset = 0.0f;
		std::vector<Pose> discr_path;
		for (const auto &connection : nav_list)
		{
			dubins::discretizeCurve(connection->path, step, offset, discr_path);
		}
		path[0].setPoints(discr_path);

		toc = std::chrono::high_resolution_clock::now();
		duration = toc - tic;
		std::cout << " DONE [" << duration.count() << " ms]" << std::endl;
	}
}
