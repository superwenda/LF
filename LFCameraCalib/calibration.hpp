#ifndef CALIBRATION_INCLUDE_HPP
#define CALIBRATION_INCLUDE_HPP

#include <vector>
#include <opencv2/core/types.hpp>

namespace anakin
{
    template <typename PType>
    void DetectCenters(cv::Mat &binMat, std::vector<PType> &centers);

    template <typename PType>
    std::vector<PType> CropBorder(std::vector<PType> centers, size_t w, size_t h, size_t left, size_t right, size_t top, size_t bottom);

    template <typename PType>
    cv::Mat DetectGridMatrix(std::vector<PType> centers, double estimate_dist = 0);

    template <typename PType>
    double EstimateAverageDist(std::vector<PType> centers);

    template <typename PType>
    int IsNeighbor(const PType &a, const PType &b, double estiname_dist, double dist_thresh);

    template <typename PType>
    void DetectCenters(cv::Mat &binMat, std::vector<PType> &centers)
    {
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierachy;

        cv::findContours(binMat, contours, hierachy, cv::CHAIN_APPROX_SIMPLE, cv::RETR_TREE);
        size_t sz = contours.size();
        for (size_t i = 0; i < sz; ++i)
        {
            if (hierachy[i][3] >= 0)
                continue;
            cv::RotatedRect r = cv::minAreaRect(contours[i]);
            centers.push_back(r.center);
        }
    }

    template <typename PType>
    std::vector<PType> CropBorder(std::vector<PType> centers, size_t w, size_t h, size_t left, size_t right, size_t top, size_t bottom)
    {
        for (auto i = centers.begin(); i != centers.end();)
            i = (i->x < left || i->x >= w - right || i->y < top || i->y >= h - bottom) ? centers.erase(i) : i + 1;
        return centers;
    }

    template <typename PType>
    cv::Mat DetectGridMatrix(std::vector<PType> centers, double estimate_dist)
    {
        if (centers.empty())
        {
            std::cerr << __FILE__ << "(" << __LINE__ << "): centers.empty()==true\n";
            return cv::Mat::zeros(2,3,CV_64F);
        }
        struct Aperture
        {
            Aperture(PType p_, int row_, int col_):p(p_), row(row_), col(col_){}
            PType p;
            int row, col;
        };
        if (abs(estimate_dist) <= DBL_EPSILON)
            estimate_dist = EstimateAverageDist(centers);
        std::sort(centers.begin(), centers.end(), [](const PType &a, const PType &b) { return (a.x + a.y) > (b.x + b.y); });
        cv::Mat A, B;
        int items_num = 0;

        std::queue<Aperture> q;
        q.push(Aperture(centers.back(), 0, 0));
        centers.pop_back();
        while (!q.empty())
        {
            Aperture aper = q.front();
            q.pop();
            items_num++;

            A.push_back(static_cast<double>(aper.col));
            A.push_back(static_cast<double>(aper.row));
            A.push_back(1.0);
            B.push_back(aper.p.x);
            B.push_back(aper.p.y);

            for (auto it = centers.begin(); it != centers.end();)
            {
                int dir = IsNeighbor(aper.p, *it, estimate_dist, estimate_dist*0.25);
                switch (dir)
                {
                case 0:
                {
                    q.push(Aperture(*it, aper.row - 1, aper.col));
                    it = centers.erase(it);
                    break;
                }
                case 1:
                {
                    q.push(Aperture(*it, aper.row + 1, aper.col));
                    it = centers.erase(it);
                    break;
                }
                case 2:
                {
                    q.push(Aperture(*it, aper.row, aper.col - 1));
                    it = centers.erase(it);
                    break;
                }
                case 3:
                {
                    q.push(Aperture(*it, aper.row, aper.col + 1));
                    it = centers.erase(it);
                    break;
                }
                default:
                    it++;
                }
            }
        }
        
        try
        {
            A = A.reshape(1, items_num);
	        B = B.reshape(1, items_num);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            std::cerr << items_num << "\n" << A.size << "\n" << B.size << "\n";
            return cv::Mat::zeros(2,3,CV_64F);
        }
        
        cv::Mat Matrix;
        cv::solve(A, B, Matrix, cv::DECOMP_SVD);
        return Matrix;
    }

    template <typename PType>
    double EstimateAverageDist(std::vector<PType> centers)
    {
        // std::sort(centers.begin(), centers.end(), [](const PType &a, const PType &b) { return (a.x + a.y) < (b.x + b.y); });
        std::vector<double> dist_vec;
        while (centers.size() > 1)
        {
            PType p = centers.back();
            centers.pop_back();
            auto it_closest = centers.begin();
            for (auto it = centers.begin(); it != centers.end(); ++it)
            {
                if (cv::norm(*it - p) < cv::norm(*it_closest - p))
                    it_closest = it;
            }
            dist_vec.push_back(cv::norm(*it_closest - p));
            centers.erase(it_closest);
        }
        std::sort(dist_vec.begin(), dist_vec.end());
        return dist_vec[dist_vec.size() / 2];
    }

    template <typename PType>
    int IsNeighbor(const PType &a, const PType &b, double estiname_dist, double dist_thresh)
    {
        PType dir[4] = {PType(0, -estiname_dist), PType(0, estiname_dist), PType(-estiname_dist, 0), PType(estiname_dist, 0)};
        for (int i = 0; i < 4; ++i)
        {
            if (cv::norm(a + dir[i] - b) < dist_thresh)
                return i;
        }
        return -1;
    }
} // namespace anakin

#endif