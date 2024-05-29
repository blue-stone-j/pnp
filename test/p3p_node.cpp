

#include <iostream>
#include "p3p.h"


int main()
{
  // 1920*1080

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 983.349f, 0, 959.5f,
                          0, 984.953f, 539.5f,
                          0, 0, 1);
  cv::Mat distCoeffs   = (cv::Mat_<double>(5, 1) << -0.0069f, -0.0174f, 0.0045f, 0, 0);
  //    cv::Mat distCoeffs = (cv::Mat_<double >(5,1) << 0,0,0,0,0);

  std::vector<cv::Point3d> objectPoints(3);
  objectPoints[0] = cv::Point3f(-1405, 260, 0);
  objectPoints[1] = cv::Point3f(-415, 354, 0);
  objectPoints[2] = cv::Point3f(-1405, 448, 0);

  std::vector<cv::Point2d> imagePoints(3);
  imagePoints[0] = cv::Point2f(506.95f, 609.08f);
  imagePoints[1] = cv::Point2f(763.5f, 623.3f);
  imagePoints[2] = cv::Point2f(511.12f, 659.56f);

  // 第4个点
  // objectPoints.emplace_back(-910,542,0);
  // imagePoints.emplace_back(634.82,681.63);

  int nSolver = 0;
  std::vector<cv::Mat> rvecs, tvecs;


  std::cout << "OpenCV's result:" << std::endl;
  std::vector<cv::Mat>().swap(rvecs);
  std::vector<cv::Mat>().swap(tvecs);
  // article: Complete Solution Classification for the Perspective-Three-Point Problem
  nSolver = cv::solveP3P(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs, cv::SOLVEPNP_P3P);
  for (int i = 0; i < nSolver; ++i)
  {
    printf("rvecs[%d]=\n", i);
    std::cout << rvecs[i] << std::endl;
  }
  for (int i = 0; i < nSolver; ++i)
  {
    printf("tvecs[%d]=\n", i);
    std::cout << tvecs[i] << std::endl;
  }

  std::cout << "\n\nour's result:" << std::endl;
  std::vector<cv::Mat>().swap(rvecs);
  std::vector<cv::Mat>().swap(tvecs);
  nSolver = SolveP3P(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecs, tvecs, cv::SOLVEPNP_P3P);
  for (int i = 0; i < nSolver; ++i)
  {
    printf("rvecs[%d]=\n", i);
    std::cout << rvecs[i] << std::endl;
  }
  for (int i = 0; i < nSolver; ++i)
  {
    printf("tvecs[%d]=\n", i);
    std::cout << tvecs[i] << std::endl;
  }

  return 0;
}
