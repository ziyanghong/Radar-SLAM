#include "cfar.h"

void detectPeakCFAR(const cv::Mat azimuth_scan, int azimuth_index, std::vector<cv::KeyPoint> &keypoints, int visPointSize,
                    const int num_train, const int num_guard, const double falseAlarmRate)
{

    /* 
    Detecet peaks with Cell Averaging CFAR algorithm

    azimuth_scan; the scan of interest
    azimuth_index; the index of azimuth angle
    keypoints: cv type keypoints
    num_train: Number of training cells.
    num_guard: Number of guard cells.
    falseAlarmRate: False alarm rate.
    */

    int num_cells = azimuth_scan.rows;
    int num_train_half = round(num_train / 2);
    int num_guard_half = round(num_guard / 2);
    int num_side = num_train_half + num_guard_half;

    double pw = pow(falseAlarmRate, (-1.0 / (double)num_train));
    //    std::cout<< "pw: " << pw << std::endl;
    double alpha = num_train * (pw - 1.0);
    //    std::cout<< "alpha: " << alpha << std::endl;

    for (int i = 20; i < (num_cells - 40); i++)
    {

        //-- boundaries
        int start = 0;
        int end = 0;
        if ((i - num_side) <= 0)
        {
            start = 0;
            end = num_side * 2;
        }
        else
        {
            start = i - num_side;
            end = i + num_side;
        }

        if ((i + num_side) >= num_cells)
        {
            start = i - num_side * 2;
            end = num_cells;
        }
        else
        {
            start = i - num_side;
            end = i + num_side;
        }
        //-- boundaries

        float sum1 = 0;
        for (int s1 = start; s1 < end; s1++)
        {
            sum1 += azimuth_scan.at<uchar>(s1, 1);
        }

        float sum2 = 0;
        for (int s2 = (i - num_guard_half); s2 < (i + num_guard_half); s2++)
        {
            sum2 += azimuth_scan.at<uchar>(s2, 1);
        }

        double p_noise = (sum1 - sum2) / num_train;
        double threshold = alpha * p_noise;
        if (azimuth_scan.at<uchar>(i, 1) > threshold)
        {
            //    std::cout<< "threshold: " << threshold << std::endl;
            //    std::cout<< "azimuth_scan.at<float>(i,1): " << azimuth_scan.at<float>(i,1) << std::endl;
            cv::Point2f pt(azimuth_index, i);
            cv::KeyPoint new_keypoint(pt, visPointSize);
            keypoints.push_back(new_keypoint);
        }
    }
}

void computeDescriptor(cv::Mat cart_image, int radius, int wedgeResolution, std::vector<cv::KeyPoint> keypoints, cv::Mat &descriptors)
{
    auto start = std::chrono::high_resolution_clock::now();
    // std::cout << "size: " << keypoints.size() << std::endl;
    // std::cout << "wedgeResolution: " << wedgeResolution << std::endl;
    descriptors.create(keypoints.size(), wedgeResolution, CV_64FC1);
    // std::cout << "descriptors rows: " << descriptors.rows << " descriptors cols: " << descriptors.cols << std::endl;

    // Construct KD tree
    pointVec points;
    point_t pt;    
    for (int i = 0; i < keypoints.size(); i++){
        pt = {(double)keypoints[i].pt.x, (double)keypoints[i].pt.y};
        points.push_back(pt); 
    }
    KDTree tree(points);

    for (int i = 0; i < keypoints.size(); i++)
    {
        int x0 = keypoints[i].pt.x;
        int y0 = keypoints[i].pt.y;

        //-- Get neighborhood indeces from the circle
        pt = {(double)x0, (double)y0};
        std::vector< size_t > neighhood_pt_index = tree.neighborhood_indices(pt, double(radius));  
        
        // Compute statistics from each wedge resolution        
        Complex mean_vec[wedgeResolution];     
         
        for (int j = 0; j < wedgeResolution; j++)
        {
            // Prepare points that are inside the wedge
            double theta_start = j * 2 * M_PI / wedgeResolution;
            double theta_end = (j + 1) * 2 * M_PI / wedgeResolution;

            int x1 = x0 + round(radius * cos(theta_start));
            int y1 = y0 - round(radius * sin(theta_start));

            int x2 = x0 + round(radius * cos(theta_end));
            int y2 = y0 - round(radius * sin(theta_end));

            // Compute mean of the feature set in the wedge
            float mean = 0.0;
            int counter = 0;
            //-- Transverse all points
            // for (int k = 0; k < keypoints.size(); k++)
            // {
            //     if (k == i)
            //     {
            //         continue;
            //     }
            //     else
            //     {
            //         int x_k = keypoints[k].pt.x;
            //         int y_k = keypoints[k].pt.y;

            //         if (isInside(x0, y0, x1, y1, x2, y2, x_k, y_k))
            //         {
            //             mean += cart_image.at<float>(x_k, y_k);
            //             counter += 1;
            //         }
            //     }
            // }

            for (int k = 0; k < neighhood_pt_index.size(); k++){
                int x_k = keypoints[neighhood_pt_index[k]].pt.x;
                int y_k = keypoints[neighhood_pt_index[k]].pt.y;
                if (isInside(x0, y0, x1, y1, x2, y2, x_k, y_k))
                    {
                        mean += cart_image.at<float>(x_k, y_k);
                        counter += 1;
                    }                
            }

            // mean_vec.at<float>(j, 0) = mean;
            mean_vec[j] = mean / (float)counter;
        }

        // /*----------------------------------------------------------------------------------------------------------------------*/
        // /* Take the fft of the statistic vector of keypoint ith.
        //    Reference: https://docs.opencv.org/3.1.0/d8/d01/tutorial_discrete_fourier_transform.html
        // */
        // cv::Mat ff;
        // std::cout << "mean_vec: " << mean_vec << std::endl;
        // cv::Mat padded;                            //expand input image to optimal size        
        // int m = cv::getOptimalDFTSize( mean_vec.rows );
        // int n = cv::getOptimalDFTSize( mean_vec.cols ); // on the border add zero values 
        // cv::copyMakeBorder(mean_vec, padded, 0, m - mean_vec.rows, 0, n - mean_vec.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));        
        // std::cout << "m: " << m << " n: " << n << std::endl;
        // cv::dft(mean_vec, ff, cv::DFT_COMPLEX_OUTPUT);
        // std::cout<< "cv::dft(mean_vec, ff, cv::DFT_COMPLEX_OUTPUT);" << std::endl;

        // //Make place for both the complex and the real values
        // cv::Mat planes[] = {cv::Mat::zeros(mean_vec.cols,1, CV_32F), cv::Mat::zeros(mean_vec.cols,1, CV_32F)};
        // std::cout << "ff rows: " << ff.rows << " ff cols: " << ff.cols << std::endl;
        // cv::split(ff, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))

        // int c = planes[0].cols;
        // int pivot = ceil(c/2);
        // //duplicate FFT results with Complex conjugate in order to get exact matlab results

        // for (int i = pivot + 1, k = pivot; i < planes[1].cols; i++, k--)
        // {
        //     planes[1].at<float>(i) = planes[1].at<float>(k) * -1; 
        //     planes[0].at<float>(i) = planes[0].at<float>(k);
        // }   

        // cv::magnitude(planes[0], planes[1], planes[0]); // planes[0] = magnitude
        // cv::Mat magI = planes[0];
        // magI += cv::Scalar::all(1); // switch to logarithmic scale
        // cv::log(magI, magI);
        // cv::normalize(magI, magI, 0, 1, cv::NORM_MINMAX); // Transform the matrix with float values into a viewable image form (float between values 0 and 1).
        // descriptors.row(i) = magI.clone();

        // /*----------------------------------------------------------------------------------------------------------------------*/
        CArray data(mean_vec, wedgeResolution);
        // forward fft
        fft(data);        
        for (int c = 0; c < wedgeResolution; ++c)
        {
            // std::cout << data[c] << std::endl;
            descriptors.at<float>(c,i) = std::abs(data[c]);
        }
    }
    
    // cv::normalize(descriptors, descriptors, 0, 1, cv::NORM_MINMAX); // Transform the matrix with float values into a viewable image form (float between values 0 and 1).

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Oxford descriptors computation time is: " << duration.count() << " milliseconds." << std::endl;

    // cv::namedWindow("Descriptors", cv::WINDOW_AUTOSIZE); // Create a window for display.
    // cv::imshow("descriptors", descriptors.t());
}

/* A utility function to calculate area of triangle formed by (x1, y1),  
   (x2, y2) and (x3, y3) */
float area(int x1, int y1, int x2, int y2, int x3, int y3)
{
    return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2.0);
}

/* A function to check whether point P(x, y) lies inside the triangle formed  
   by A(x1, y1), B(x2, y2) and C(x3, y3) */
bool isInside(int x1, int y1, int x2, int y2, int x3, int y3, int x, int y)
{
    /* Calculate area of triangle ABC */
    float A = area(x1, y1, x2, y2, x3, y3);

    /* Calculate area of triangle PBC */
    float A1 = area(x, y, x2, y2, x3, y3);

    /* Calculate area of triangle PAC */
    float A2 = area(x1, y1, x, y, x3, y3);

    /* Calculate area of triangle PAB */
    float A3 = area(x1, y1, x2, y2, x, y);

    /* Check if sum of A1, A2 and A3 is same as A */
    return (A == A1 + A2 + A3);
}


/* Reference: https://rosettacode.org/wiki/Fast_Fourier_transform#C.2B.2B 
// Cooley–Tukey FFT (in-place, divide-and-conquer)
// Higher memory requirements and redundancy although more intuitive*/
void fft(CArray& x)
{
    const size_t N = x.size();
    if (N <= 1) return;
 
    // divide
    CArray even = x[std::slice(0, N/2, 2)];
    CArray  odd = x[std::slice(1, N/2, 2)];
 
    // conquer
    fft(even);
    fft(odd);
 
    // combine
    for (size_t k = 0; k < N/2; ++k)
    {
        Complex t = std::polar(1.0, -2 * PI * k / N) * odd[k];
        x[k    ] = even[k] + t;
        x[k+N/2] = even[k] - t;
    }
}