#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>

#include <vector>
#include <iostream>
#include <chrono>
#include <random>

#include "rm/RoadMap.hpp"
#include "rm/visibility.hpp"
#include "rm/inflate.hpp"
#include "nav/NavMap.hpp"
#include "nav/pursuerEvader.hpp"
#include "nav/path.hpp"
#include "dubins/dubins.hpp"
#include "utils/timer.hpp"
#include "utils/MatlabPlot.hpp"

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
		const float robot_size = 0.14f;							 // Width of the robot (wheel-wheel)
		const float collision_offset = robot_size * 0.5f;		 // Offset for obstacle inflation
		const float visibility_offset = collision_offset * 1.3f; // Offset for visibility graph vertices
		const float visibility_threshold = robot_size * 0.5f;	 // Minimum distance between consecutive nodes
		const int n_poses = 8;									 // Number of poses per node
		const float kmax = 1 / robot_size;						 // Maximum curvature of Dubins paths
		const int k = 10; 										 // Robot free roaming parameter
		const float step = M_PI / 32 / kmax;					 // Discretization step
		const bool enable_matlab_output = true; 				 // Whether to generate matlab file for plotting
		const std::string matlab_file = config_folder + "/student_interface_plot.m";

		std::cout << "**********************************************************************" << std::endl;
		std::cout << "*   PURSUER-EVADER ROBOT GAME - ROADMAP BUILDING & MOTION PLANNING   *" << std::endl;
		std::cout << "*                  by Simone Peroni & Mario Tilocca                  *" << std::endl;
		std::cout << "**********************************************************************" << std::endl;
		std::cout << "Evader initial pose:  (x: " << x[0] << ", y: " << y[0] << ", yaw: " << theta[0] << ")" << std::endl;
		std::cout << "Pursuer initial pose: (x: " << x[1] << ", y: " << y[1] << ", yaw: " << theta[1] << ")" << std::endl;
		std::cout << "Detected obstacles:   " << obstacle_list.size() << std::endl;
		std::cout << "Detected gates:       " << gate_list.size() << std::endl;
		if (enable_matlab_output)
			std::cout << "Matlab output file:   " << matlab_file << std::endl;
		std::cout << "**********************************************************************" << std::endl;
		std::cout << "Running algorithm..." << std::endl
				  << std::endl;

		utils::Timer t;

		try
		{
			// Inflate obstacles and borders
			t.tic("Inflating obstacles and borders...");
			auto infObstacles = rm::inflate(obstacle_list, collision_offset, true);
			auto infBorders = rm::inflate(std::vector<Polygon>{borders}, -collision_offset, false).back();
			t.toc();

			// Select vertices
			t.tic("Selecting vertices for graph...");
			std::vector<Point> vertices;
			rm::makeVisibilityNodes(obstacle_list, borders, visibility_offset, vertices, visibility_threshold);
			t.toc();

			// Setup RoadMap by visibility graph
			t.tic("Computing visibility graph...");
			rm::RoadMap rm;
			rm::visibility(rm, vertices, infObstacles, infBorders);
			t.toc();

			// Build RoadMap
			t.tic("Building roadmap...");
			rm.build(n_poses, kmax, infObstacles, infBorders);
			t.toc();

			// Add initial positions
			t.tic("Adding evader start pose...");
			auto &source_e = rm.addStartPose(Point(x[0], y[0]), theta[0], k, kmax, infObstacles, infBorders);
			t.toc();

			t.tic("Adding pursuer start pose...");
			auto &source_p = rm.addStartPose(Point(x[1], y[1]), theta[1], k, kmax, infObstacles, infBorders);
			t.toc();

			// Add gate positions
			t.tic("Adding goal poses...");
			std::vector<rm::RoadMap::Node::Orientation *> goal;
			for (size_t i = 0; i < gate_list.size(); i++)
			{
				t.tic();
				float gate_x, gate_y, gate_th;
				rm::getGatePose(gate_list[i], borders, gate_x, gate_y, gate_th);
				goal.push_back(&rm.addGoalPose(Point(gate_x, gate_y), gate_th, k, kmax, infObstacles, borders));
				t.toc(std::to_string(i + 1) + "/" + std::to_string(gate_list.size()));
			}
			t.toc();

			// Precompute navigation weights
			t.tic("Precomputing navigation maps for evader (Dijkstra algorithm)...");
			std::vector<nav::NavMap> nm_e;
			for (size_t i = 0; i < goal.size(); i++)
			{
				t.tic();
				nm_e.push_back(nav::NavMap(rm));
				nm_e.back().computeReverse(*goal[i]);
				t.toc(std::to_string(i + 1) + "/" + std::to_string(goal.size()));
			}
			t.toc();

			// Run game
			t.tic("Running game...");
			nav::navList nav_list_e, nav_list_p;
			nav::NavMap nm_p(rm);
			nav::runGame(nm_e, nm_p, source_e, source_p, nav_list_e, nav_list_p);
			t.toc();

			// Discretize paths
			t.tic("Discretizing paths...");
			t.tic();
			std::vector<Pose> discr_path_e;
			nav::discretizePath(nav_list_e, step, discr_path_e);
			t.toc("Evader path");

			t.tic();
			std::vector<Pose> discr_path_p;
			nav::discretizePath(nav_list_p, step, discr_path_p);
			t.toc("Pursuer path");
			t.toc();

			// Truncate paths at collision point
			t.tic("Truncating paths at collision point");
			nav::truncatePaths(discr_path_e, discr_path_p, robot_size);
			path[0].setPoints(discr_path_e);
			path[1].setPoints(discr_path_p);
			t.toc();

			// Create matlab file for plotting
			if (enable_matlab_output)
			{
				t.tic("Creating matlab file...");
				utils::MatlabPlot mp(matlab_file.data());
				mp.plotPolygons(obstacle_list, "k-");
				mp.plotPolygon(borders, "k-");
				mp.plotPolygons(infObstacles);
				mp.plotPolygon(infBorders);
				mp.plotPolygons(gate_list, "g-");
				mp.plotGraph(rm);
				mp.plotDiscretePath(discr_path_e);
				mp.plotDiscretePath(discr_path_p, "k.");
				t.toc();
			}

			std::cout << std::endl
					  << "Algorithm terminated succesfully" << std::endl;
			std::cout << "**********************************************************************" << std::endl;
		}
		catch (const std::logic_error &e)
		{
			std::cout << std::endl
					  << "Algorithm stopped! Exception raised: " << e.what() << std::endl;
			std::cout << "**********************************************************************" << std::endl;
			return false;
		}
		return true;
	}
}
