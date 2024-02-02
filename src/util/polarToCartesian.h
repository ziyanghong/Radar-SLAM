#ifndef POLARTOCARTESIAN_H
#define POLARTOCARTESIAN_H

#include <iostream>
#include <string>
#include <math.h>
#include <opencv2/opencv.hpp>



static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat &X,
                     cv::Mat &Y);
                     
void polar_to_cartesian(const cv::Mat &im, cv::Mat &output);

void polar_to_cartesian(const cv::Mat img_polar, const cv::Mat img_cartesin,
const std::vector<cv::KeyPoint> KeyPointsPolar, std::vector<cv::KeyPoint> &KeyPointsCartesian);

#endif