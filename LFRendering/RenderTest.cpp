#include <opencv2/opencv.hpp>

#include "render.hpp"
#include <cmath>
int main(int argc, char *argv[])
{
    cv::FileStorage fs("D:/lib/Microview/BIN64/calibration.yaml", cv::FileStorage::READ);
    cv::Mat grid_matrix;
    int height, width;
    double diameter;

    fs["GridMatrix"] >> grid_matrix;
    fs["ImageWidth"] >> width;
    fs["ImageHeight"] >> height;
    fs["Diameter"] >> diameter;
    std::cout << grid_matrix << "\n";
    std::cout << width << "\n";
    std::cout << height << "\n";
    std::cout << diameter << "\n";
    anakin::Render r(grid_matrix, width, height, diameter, 0.5);


    cv::Mat src_img = cv::imread("D:/lib/Microview/BIN64/DEV#0_RGB_335.bmp");
    cv::imshow("Rand", src_img);
    cv::waitKey();
    double zoom = 1.0;
    while (true)
    {
        std::chrono::time_point<std::chrono::steady_clock> tp_begin = std::chrono::steady_clock::now();
        cv::Mat dst_image = r.render(src_img, zoom, 0.5, 0, 0);
        std::chrono::time_point<std::chrono::steady_clock> tp_end = std::chrono::steady_clock::now();
        std::cout << "Cost " << (std::chrono::duration_cast<std::chrono::microseconds>(tp_end - tp_begin).count()) << " microsecond(s)\n";
        int ratio = dst_image.cols / 900;
        cv::Size sz(dst_image.cols / ratio, dst_image.rows / ratio);
        cv::resize(dst_image, dst_image, sz);

        cv::imshow("Render", dst_image);
        char key = cv::waitKey();
        if (key == 27)
            break;
        else if (key == 'q')
            zoom = std::max(1.5, zoom - 0.05);
        else if (key == 'w')
            zoom = std::min(3.0, zoom + 0.05);
    }
    
}