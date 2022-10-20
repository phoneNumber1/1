#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream> 


int main(int argc, char** argv)
{
    size_t i = 0;
    std::string strglob, afpPreset, filteringPreset, channelPreset, coefsPath, fileName;
    std::cout << "Choose on of these presets 'visible' or 'infrared', then copy and paste it" << std::endl;
    std::cin >> channelPreset;
    if (channelPreset == "infrared")
    {
        std::cout << "Choose presets, copy and paste it in that order: 'affine&perspective', 'filter'\n";
        std::cin >> afpPreset;
        std::cin >> filteringPreset;
        coefsPath = "J:\\txts\\ir\\";
        strglob = "J:/diplomphoto/original/ir/cam*.bmp";
    }
    else if (channelPreset == "visible")
    {
        coefsPath = "J:\\txts\\vis\\";
        strglob = "J:/diplomphoto/original/vis/cam*.bmp";
    }
	(void)argc;
	(void)argv;
    cv::Size frameSizeIR(320, 240);
    cv::Size frameSizeVIS(1920, 1080);
    std::ifstream F1;
    float Zf1[3][3] = {};
    F1.open(coefsPath + "intrinsic_parameters.txt");
    cv::Matx33f K;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            if (F1.is_open())
            {
                F1 >> Zf1[i][j];
                K(i, j) = Zf1[i][j];
            }
        }
    }
    std::ifstream F3;
    float Zf3[5] = {};
    F3.open(coefsPath + "distortion_coefficients.txt");
    cv::Vec<float, 5> k;
    for (int i = 0; i < 5; i++)
    {
        if (F3.is_open())
        {
            F3 >> Zf3[i];
            k(i) = Zf3[i];
        }
    }
    std::vector<cv::String> fileNames;
    cv::glob(strglob, fileNames, false);
    cv::Mat mapX, mapY;
    if (channelPreset == "infrared")
    {
        cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSizeIR, CV_32FC1, mapX, mapY);
    }
    else if (channelPreset == "visible")
    {
        cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSizeVIS, CV_32FC1, mapX, mapY);
    }
    for (auto const& f : fileNames) {
        std::cout << std::string(f) << std::endl;
        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
        cv::Mat imgUndistorted; cv::Mat imgw; cv::Mat imgP, imgf, imgfs;
        if(channelPreset == "infrared") 
        {
            if (afpPreset == "affine&perspective")
            {
                cv::warpAffine(img, imgw, cv::getRotationMatrix2D(cv::Point2f(160, 120), 1.8, 1), cv::Size());
                cv::imshow("warp", imgw);
                cv::waitKey(0);
                cv::Point2f srcPoints[] =
                {
                    cv::Point2f(0,0),
                    cv::Point2f(239,0),
                    cv::Point2f(0,319),
                    cv::Point2f(239,319)
                };
                cv::Point2f dstPoints[] =
                {
                    cv::Point2f(4,6),
                    cv::Point2f(235,6),
                    cv::Point2f(0,319),
                    cv::Point2f(239,319)
                };
                cv::warpPerspective(imgw, imgP, cv::getPerspectiveTransform(srcPoints, dstPoints), cv::Size());
                cv::imshow("warp2", imgP);
                cv::waitKey(0);
                cv::remap(imgP, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
                cv::imshow("undistorted image", imgUndistorted);
                cv::waitKey(0);
                int namecounter = i + 1;
                std::stringstream ss;
                ss << namecounter;
                std::string nameCounter = ss.str();
                std::string path = "J:/diplomphoto/calibrated/ir/";
                std::string name = "cam_frame";
                cv::imwrite(path + name + nameCounter + ".bmp", imgUndistorted);
                i++;
                if (filteringPreset == "filter")
                {
                    cv::bilateralFilter(imgUndistorted, imgf, 10, 100, 100);
                    cv::imshow("filtered", imgf);
                    cv::waitKey(0);
                    cv::Matx33f M = { 0,-1,0,-1,5,-1,0,-1,0 };
                    cv::filter2D(imgf, imgfs, -1, M);
                    cv::imshow("sharpened", imgfs);
                    cv::waitKey(0);
                    cv::imwrite(path + name + "_filtered" + nameCounter + ".bmp", imgfs);
                }
            }
            else if (afpPreset != "affine&perspective")
            {
                cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
                cv::imshow("undistorted image", imgUndistorted);
                cv::waitKey(0);
                int namecounter = i + 1;
                std::stringstream ss;
                ss << namecounter;
                std::string nameCounter = ss.str();
                std::string path = "J:/diplomphoto/calibrated/ir/";
                std::string name = "cam_frame";
                cv::imwrite(path + name + nameCounter + ".bmp", imgUndistorted);
                i++;
                if (filteringPreset == "filter")
                {
                    cv::bilateralFilter(imgUndistorted, imgf, 10, 100, 100);
                    cv::imshow("filtered", imgf);
                    cv::waitKey(0);
                    cv::Matx33f M = { 0,-1,0,-1,5,-1,0,-1,0 };
                    cv::filter2D(imgf, imgfs, -1, M);
                    cv::imshow("sharpened", imgfs);
                    cv::waitKey(0);
                    cv::imwrite(path + name + "_filtered" + nameCounter + ".bmp", imgfs);
                }
            }
        }
        else if (channelPreset == "visible")
        {
            cv::warpAffine(img, imgw, cv::getRotationMatrix2D(cv::Point2f(160, 120), 0.8, 1), cv::Size());
            cv::remap(imgw, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
            cv::imshow("undistorted image", imgUndistorted);
            cv::waitKey(0);
            int namecounter = i + 1;
            std::stringstream ss;
            ss << namecounter;
            std::string nameCounter = ss.str();
            std::string path = "J:/diplomphoto/calibrated/vis/";
            std::string name = "cam_frame";
            cv::imwrite(path + name + nameCounter + ".bmp", imgUndistorted);
            i++;
        }
    }
    return 0;
}