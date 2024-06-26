#ifndef epnp_h
#define epnp_h

#include <opencv2/opencv.hpp>

#include <vector>

// "equation 5" in my comment means equation 5 in this article
class EPnP
{
 public:
  // (*, obj pts, img pts)
  EPnP(const cv::Mat &cameraMatrix, const cv::Mat &opoints, const cv::Mat &ipoints);
  ~EPnP();

  void add_correspondence(const double X, const double Y, const double Z,
                          const double u, const double v);

  // entry function
  void compute_pose(cv::Mat &R, cv::Mat &t);

 private:
  EPnP(const EPnP &);            // copy disabled
  EPnP &operator=(const EPnP &); // assign disabled
  template <typename T>
  void init_camera_parameters(const cv::Mat &cameraMatrix)
  {
    uc = cameraMatrix.at<T>(0, 2);
    vc = cameraMatrix.at<T>(1, 2);
    fu = cameraMatrix.at<T>(0, 0);
    fv = cameraMatrix.at<T>(1, 1);
  }
  // extract data from mat
  template <typename OpointType, typename IpointType>
  void init_points(const cv::Mat &opoints, const cv::Mat &ipoints)
  {
    for (int i = 0; i < number_of_correspondences; i++)
    {
      pws[3 * i]     = opoints.at<OpointType>(i).x;
      pws[3 * i + 1] = opoints.at<OpointType>(i).y;
      pws[3 * i + 2] = opoints.at<OpointType>(i).z;

      us[2 * i]     = ipoints.at<IpointType>(i).x * fu + uc;
      us[2 * i + 1] = ipoints.at<IpointType>(i).y * fv + vc;
    }
  }
  double reprojection_error(const double R[3][3], const double t[3]);
  void choose_control_points(void);
  // convert coordinate from using common basis to using basis that consist of control points
  void compute_barycentric_coordinates(void);
  // (out M, in row, in new coordinate, in u, in v): uv are pixel coordinate;
  // project a point to camera
  void fill_M(cv::Mat M, const int row, const double *alphas, const double u, const double v);
  // 控制点在摄像头参考坐标下的坐标
  void compute_ccs(const double *betas, const double *ut);
  // 3D参考点在摄像头参考坐标系下的坐标 = 3D参考点在控制点坐标系下的坐标 * 控制点在摄像头参考坐标下的坐标
  void compute_pcs(void);
  //
  void solve_for_sign(void);

  // approximate value
  void find_betas_approx_1(const cv::Mat L_6x10, const cv::Mat Rho, double *betas);
  void find_betas_approx_2(const cv::Mat L_6x10, const cv::Mat Rho, double *betas);
  void find_betas_approx_3(const cv::Mat L_6x10, const cv::Mat Rho, double *betas);
  void qr_solve(cv::Mat A, cv::Mat b, cv::Mat X);

  double dot(const double *v1, const double *v2);
  double dist2(const double *p1, const double *p2);

  // distances between control points in world frame
  void compute_rho(double *rho);
  // Calculate l in case N = 4. l: left term of equation is square of distance in camera frame.
  void compute_L_6x10(const double *ut, double *l_6x10);

  void gauss_newton(const cv::Mat L_6x10, const cv::Mat Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double *l_6x10, const double *rho, const double cb[4], cv::Mat A, cv::Mat b);

  double compute_R_and_t(const double *ut, const double *betas, double R[3][3], double t[3]);
  // point in world = camera in world * point in camera
  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3], double R_src[3][3], double t_src[3]);

  double uc, vc, fu, fv; // cx; cy; fx; fy

  std::vector<double> pws;       // store all obj pts;
  std::vector<double> us;        // store all pixel coordinate;
  std::vector<double> alphas;    // coordinates of pw in the set of basis from control points;
  std::vector<double> pcs;       // 3D参考点在摄像头参考坐标系下的坐标
  int number_of_correspondences; // num of obj-img pairs

  double cws[4][3]; // control points in world ;
  double ccs[4][3]; // control points in camera
  int max_nr;       // ???
  double *A1, *A2;  // ???
};

#endif
