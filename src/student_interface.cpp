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
		const float collision_offset = robot_size / 2.0f;		 // Offset for obstacle inflation
		const float visibility_offset = collision_offset * 1.25f; // Offset for visibility graph vertices
		const float visibility_threshold = robot_size / 2.0f;	 // Minimum distance between consecutive nodes
		const int n_poses = 8;									 // Number of poses per node
		const float kmax = 1 / robot_size;						 // Maximum curvature of Dubins paths
		const int k = 50;										 // Robot free movement parameter
		const float step = M_PI / 32 / kmax;					 // Discretization step

		const bool enable_matlab_output = true; // Whether to generate matlab file for plotting
		const std::string matlab_file = config_folder + "/student_interface_plot.m";

		std::cout << "**********************************************************************" << std::endl;
		std::cout << "*   PURSUER-EVADER ROBOT GAME - ROADMAP BUILDING & MOTION PLANNING   *" << std::endl;
		std::cout << "*                  by Simone Peroni & Mario Tilocca                  *" << std::endl;
		std::cout << "**********************************************************************" << std::endl;
		std::cout << "Evader initial pose:  (x: " << x[0] << ", y: " << y[0] << ", yaw: " << theta[0] << ")" << std::endl;
		std::cout << "Pursuer initial pose: (x: " << x[0] << ", y: " << y[0] << ", yaw: " << theta[0] << ")" << std::endl;
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

			// Add initial position
			t.tic("Adding evader start pose...");
			auto &source_e = rm.addStartPose(Point(x[0], y[0]), theta[0], k, kmax, infObstacles, infBorders);
			t.toc();

			t.tic("Adding pursuer start pose...");
			auto &source_p = rm.addStartPose(Point(x[1], y[1]), theta[1], k, kmax, infObstacles, infBorders);
			t.toc();

			// Add gate position
			t.tic("Adding goal pose...");
			float gate_x = 0.0f;
			float gate_y = 0.0f;
			for (auto &p : gate_list[0])
			{
				gate_x += p.x;
				gate_y += p.y;
			}
			gate_x /= gate_list[0].size();
			gate_y /= gate_list[0].size();
			auto &goal = rm.addGoalPose(Point(gate_x, gate_y), M_PI, k, kmax, infObstacles, borders);
			t.toc();

			// Precompute navigation weights
			t.tic("Precomputing navigation map for evader (Dijkstra algorithm)...");
			nav::NavMap nm_e(rm);
			nm_e.computeReverse(goal);
			t.toc();

			t.tic("Precomputing navigation map for pursuer (Dijkstra algorithm)...");
			nav::NavMap nm_p(rm);
			nm_p.compute(source_p);
			t.toc();

			// Get shortest path from source
			t.tic("Acquiring best escape path...");
			nav::navList nav_list_e = nm_e.planFrom(source_e);
			t.toc();

			t.tic("Intercepting evader escape path...");
			nav::navList nav_list_p = nm_p.intercept(nav_list_e);
			t.toc();

			// Discretize path
			t.tic("Discretizing path for evader...");
			float offset = 0.0f;
			std::vector<Pose> discr_path_e;
			for (const auto &connection : nav_list_e)
			{
				dubins::discretizeCurve(connection->path, step, offset, discr_path_e);
			}
			path[0].setPoints(discr_path_e);
			t.toc();

			t.tic("Discretizing path for pursuer...");
			offset = 0.0f;
			std::vector<Pose> discr_path_p;
			for (const auto &connection : nav_list_p)
			{
				dubins::discretizeCurve(connection->path, step, offset, discr_path_p);
			}
			path[1].setPoints(discr_path_p);
			t.toc();

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

			std::cout << std::endl << "Algorithm terminated succesfully" << std::endl;
			std::cout << "**********************************************************************" << std::endl;
		}
		catch (const std::logic_error &e)
		{
			std::cout << std::endl << "Algorithm stopped! Exception raised: " << e.what() << std::endl;
			std::cout << "**********************************************************************" << std::endl;
			return false;
		}
		return true;
	}
}
