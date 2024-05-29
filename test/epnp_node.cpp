

#include <iostream>
#include "epnp.h"

int main(int argc, char **argv)
{
  cv::Mat cameraMatrix;
  cv::Mat opoints;
  cv::Mat ipoints;
  EPnP epnp(cameraMatrix, opoints, ipoints);
}