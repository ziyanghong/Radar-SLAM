#include "polarToCartesian.h"

static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv, cv::Mat &X,
                     cv::Mat &Y)
{
  cv::repeat(xgv.reshape(1, 1), ygv.total(), 1, X);
  cv::repeat(ygv.reshape(1, 1).t(), 1, xgv.total(), Y);
}

void remap(const cv::Mat &im, const cv::Mat &x, const cv::Mat &y,
           cv::Mat &output, const int width)
{
  float ratio = 1.0;
  int height = round(width * ratio);
  double min_x, max_x;
  cv::minMaxLoc(x, &min_x, &max_x);
  cv::Mat div_x;
  cv::divide((x - min_x), (max_x - min_x), div_x);
  cv::Mat xx = (div_x * (height - 1));

  double min_y, max_y;
  cv::minMaxLoc(y, &min_y, &max_y);
  cv::Mat div_y;
  cv::divide((y - min_y), (max_y - min_y), div_y);
  cv::Mat yy = (div_y * (width - 1));

  output = cv::Mat::zeros(height, width, CV_8UC1);
  for (int i = 0; i < im.rows; i++)
  {
    for (int j = 0; j < im.cols; j++)
    {
      output.at<uchar>(static_cast<int>(floor(xx.at<float>(i, j))),
                       static_cast<int>(floor(yy.at<float>(i, j)))) =
          im.at<uchar>(i, j);
    }
  }
}

void polar_to_cartesian(const cv::Mat &im, cv::Mat &output)
{
  cv::Mat X, Y;
  int ROWS = 576;
  int COLS = 400;
  int size = ROWS * 2;

  cv::Mat theta = cv::Mat::zeros(1, im.cols, CV_32FC1);

  cv::Mat range = cv::Mat::zeros(1, ROWS, CV_32FC1);

  for (int i = 0; i < im.cols; i++)
  {
    theta.at<float>(0, i) = (i * 2.0 * M_PI) / im.cols;
  }

  for (int i = 0; i < ROWS; i++)
  {
    range.at<float>(0, i) = i;
  }

  cv::Mat angles;
  cv::Mat ranges;
  meshgrid(theta, range, angles, ranges);
  cv::polarToCart(ranges, angles, X, Y);
  remap(im, X, Y, output, size);
}

void polar_to_cartesian(const cv::Mat img_polar, const cv::Mat img_cartesin,
                        const std::vector<cv::KeyPoint> KeyPointsPolar, std::vector<cv::KeyPoint> &KeyPointsCartesian)
{

  int ROWS = img_polar.rows;
  int COLS = img_polar.cols;

  // std::cout<< "ROWS: " << ROWS << std::endl;
  for (int i = 0; i < KeyPointsPolar.size(); i++)
  {
    cv::Point2f pt = KeyPointsPolar[i].pt;
    
    double theta = (pt.x * 2.0 * M_PI) / COLS; 
    double range = pt.y;

    int x = ROWS - 1 + round(range*sin(theta));
    int y = ROWS - 1 - round(range*cos(theta)) ;
    cv::Point2f _pt(x,y);
    cv::KeyPoint new_keypoint(_pt, 5);
    KeyPointsCartesian.push_back(new_keypoint);  
  }
}
