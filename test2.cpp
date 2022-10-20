#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <fstream> 

int main(int argc, char** argv) {

    (void)argc;
    (void)argv;
    std::string preSet;
    std::cout << "Choose preset for calibration 'visible' or 'infrared', copy and paste chosen preset:\n";
    std::cin >> preSet;
    std::string Path;
    int patternSize1 = 8, patternSize2 = 5, fieldSize = 50, frameSize1, frameSize2;
    if (preSet == "visible")
    {
        Path = "E:/forotchet/vis/work/cam*.bmp";
        frameSize1 = 1920;
        frameSize2 = 1080;
    }
    else if (preSet == "infrared")
    {
        Path = "E:/forotchet/ir/negs/work/cam*.bmp";
        frameSize1 = 320;
        frameSize2 = 240;
    }
    std::vector<cv::String> fileNames;
    cv::glob(Path, fileNames, false);
    cv::Size patternSize(patternSize1 - 1, patternSize2 - 1);
    std::vector<std::vector<cv::Point2f>> q(fileNames.size());
    std::vector<std::vector<cv::Point3f>> Q;
    int checkerBoard[2] = { patternSize1,patternSize2 };
    std::vector<cv::Point3f> objp;
    for (int i = 1; i < checkerBoard[1]; i++) {
        for (int j = 1; j < checkerBoard[0]; j++) {
            objp.push_back(cv::Point3f(j*fieldSize, i*fieldSize, 0));
        }
    }
    std::vector<cv::Point2f> imgPoint;
    std::size_t i = 0;
    for (auto const& f : fileNames) {
        cv::Mat img = cv::imread(fileNames[i]);
        cv::Mat gray, gray2;
        if (preSet == "infrared")
        {
            cv::cvtColor(img, gray2, cv::COLOR_RGB2GRAY);
            gray = 255 - gray2;
            bool patternFound = cv::findChessboardCornersSB(gray, patternSize, q[i], cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
            if (patternFound) {
                std::cout << std::string(f) << std::endl;
                //cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                Q.push_back(objp);
            }
            cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
            cv::imshow("chessboard detection", img);
            cv::waitKey(0);
        }
        else if (preSet == "visible")
        {
            cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
            bool patternFound = cv::findChessboardCornersSB(gray, patternSize, q[i], cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_EXHAUSTIVE + cv::CALIB_CB_ACCURACY);
            if (patternFound) {
                std::cout << std::string(f) << std::endl;
                //cv::cornerSubPix(gray, q[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
                Q.push_back(objp);
            }
            cv::drawChessboardCorners(img, patternSize, q[i], patternFound);
            cv::imshow("chessboard detection", img);
            cv::waitKey(0);
        }
        i++;
    }
    cv::Matx33f K(cv::Matx33f::eye());
    cv::Vec<float, 5> k(0, 0, 0, 0, 0);

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(frameSize1, frameSize2);
    std::cout << "Calibrating..." << std::endl;
    double error = cv::calibrateCamera(Q, q, frameSize, K, k, rvecs, tvecs, flags, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT + cv::TermCriteria::MAX_ITER, 30, 0.1));
      std::cout << "Reprojection error = " << error << "\nK =\n"
        << K << "\nk=\n"
        << k << std::endl;
    cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
        mapX, mapY);
    for (auto const& f : fileNames) {
        std::cout << std::string(f) << std::endl;
        cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
        cv::Mat imgUndistorted;
        cv::remap(img, imgUndistorted, mapX, mapY, cv::INTER_LINEAR);
        cv::imshow("undistorted image", imgUndistorted);
        cv::waitKey(0);
    }
    std::ofstream F;
    F.open("D:\\txts\\ir\\intrinsic_parameters.txt");
    float Zf[3][3] = {};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Zf[i][j] = K(i, j);
            if (F.is_open())
            {
                F << Zf[i][j] << std::endl;
            }
        }
    }
    std::ofstream F2;
    F2.open("D:\\txts\\ir\\distortion_coefficients.txt");
    float Zf2[5] = {};
    for (int i = 0; i < 5; i++)
    {
            Zf2[i] = k(i);
            if (F2.is_open())
            {
                F2 << Zf2[i] << std::endl;
            }
    }
    return 0;
}
