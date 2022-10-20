#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <fstream>

using namespace std;
using namespace cv;

int main()
{
    string pathIR, pathVis;
    cout << "Put the path to vis photo like 'J:/diplomphoto/calibrated/vis/cam_frame6.bmp':\n";
    cin >> pathVis;
    cout << "Put the path to IR photo like 'J:/diplomphoto/calibrated/ir/cam_frame3.bmp':\n";
    cin >> pathIR;
    Mat imgvis = imread(pathVis, CV_64F);
    Mat imgir = imread(pathIR, CV_64F);
    Mat imgvisg, imgirg, imgvisgc, imgirgsc, imgfused;
    cvtColor(imgvis, imgvisg, COLOR_RGB2GRAY);
    cvtColor(imgir, imgirg, COLOR_RGB2GRAY);
    Mat imgirgs(902, 1193, CV_64F);
    resize(imgirg, imgirgs, imgirgs.size());
    imgvisgc = imgvisg(Range(192, 1080), Range(418, 1611));
    imgirgsc = imgirgs(Range(0, 888), Range(0, 1193));
    imshow("gray vis image cropped", imgvisgc);
    imwrite("J:/diplomphoto/fused/cropped_gray_vis.bmp", imgvisgc);
    waitKey(0);
    imshow("scaled ir image cropped", imgirgsc);
    imwrite("J:/diplomphoto/fused/scaled_cropped_ir.bmp", imgirgsc);
    waitKey(0);
    imgvisgc.convertTo(imgvisgc, CV_64F);
    imgirgsc.convertTo(imgirgsc, CV_64F);
    imgvisgc = imgvisgc / 255;
    imgirgsc = imgirgsc / 255;
    ///////////////////////////////////////////////////////weighted sum
    double c = 0;    double g = 0;    double c1 = 0;
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            c = c + imgvisgc.at<double>(j, i) - imgirgsc.at<double>(j, i);
            c1 = c1 + imgirgsc.at<double>(j, i);
            g = g + imgvisgc.at<double>(j, i) + imgirgsc.at<double>(j, i);
        }
    }
    double alpha = abs(c / g);
    double beta = 1 - alpha;
    double gamma = 0;
    double T = c1 / g;
    addWeighted(imgvisgc, alpha, imgirgsc, beta, gamma, imgfused);
    imshow("weighted sum", imgfused);
    //imwrite("J:/diplomphoto/fused/weighted_sum.bmp", imgfused * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////mean
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            imgfused.at<double>(j, i) = (imgvisgc.at<double>(j, i) + imgirgsc.at<double>(j, i)) / 2;
        }
    }
    imshow("medium", imgfused);
    imwrite("J:/diplomphoto/fused/mean.bmp", imgfused * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////maximum
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            if (imgvisgc.at<double>(j, i) >= imgirgsc.at<double>(j, i))
            {
                imgfused.at<double>(j, i) = imgvisgc.at<double>(j, i);
            }
            else if (imgvisgc.at<double>(j, i) < imgirgsc.at<double>(j, i))
            {
                imgfused.at<double>(j, i) = imgirgsc.at<double>(j, i);
            }
        }
    }
    imshow("maximum", imgfused);
    imwrite("J:/diplomphoto/fused/max.bmp", imgfused * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////mask
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            if (imgvisgc.at<double>(j, i) >= T)
            {
                imgfused.at<double>(j, i) = imgvisgc.at<double>(j, i);
            }
            else if (imgvisgc.at<double>(j, i) < T)
            {
                if (imgirgsc.at<double>(j, i) > imgvisgc.at<double>(j, i))
                {
                    imgfused.at<double>(j, i) = imgirgsc.at<double>(j, i);
                }
                else if (imgirgsc.at<double>(j, i) <= imgvisgc.at<double>(j, i))
                {
                    imgfused.at<double>(j, i) = imgvisgc.at<double>(j, i);
                }
            }
        }
    }
    imshow("mask", imgfused);
    imwrite("J:/diplomphoto/fused/mask.bmp", imgfused * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////powered
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            imgfused.at<double>(j, i) = pow((imgirgsc.at<double>(j, i)), (1 - imgvisgc.at<double>(j, i)));
        }
    }
    imshow("powered", imgfused);
    imwrite("J:/diplomphoto/fused/power_transform.bmp", imgfused * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////rowthroughv1
    Mat imgfusedt(imgvisgc.rows * 2, imgvisgc.cols, CV_64F);
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            imgfusedt.at<double>(2 * j, i) = imgirgsc.at<double>(j, i);
            imgfusedt.at<double>(2 * j + 1, i) = imgvisgc.at<double>(j, i);
        }
    }
    imshow("rowthrough", imgfusedt);
    imwrite("J:/diplomphoto/fused/interlacedv1.bmp", imgfusedt * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////rowthroughv2
    for (int j = 0; j <= imgirgsc.rows - 2; j += 2)
    {
        for (int i = 0; i <= imgirgsc.cols - 1; i++)
        {
            imgfused.at<double>(j, i) = imgirgsc.at<double>(j, i);
            imgfused.at<double>(j + 1, i) = imgvisgc.at<double>(j, i);
        }
    }
    imshow("rowthroughv2", imgfused);
    imwrite("J:/diplomphoto/fused/interlacedv2.bmp", imgfused * 255);
    waitKey(0);
    ///////////////////////////////////////////////////////chessboard
    for (int j = 0; j <= imgirgsc.rows - 1; j++)
    {
        for (int i = 0; i <= imgirgsc.cols - 2; i++)
        {
            if ((i + j) % 2 == 0)
            {
                imgfused.at<double>(j, i) = imgirgsc.at<double>(j, i);
                imgfused.at<double>(j, i + 1) = imgvisgc.at<double>(j, i);
            }
            else if ((i + j) % 2 != 0)
            {
                imgfused.at<double>(j, i + 1) = imgirgsc.at<double>(j, i);
                imgfused.at<double>(j, i) = imgvisgc.at<double>(j, i);
            }
        }
    }
    imshow("chessboard", imgfused);
    imwrite("J:/diplomphoto/fused/chessboard.bmp", imgfused * 255);
    waitKey(0);
    return 0;
}