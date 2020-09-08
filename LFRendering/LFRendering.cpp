#include <afxwin.h>
#include <MVCAPI.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>

void getSourcePoints(const cv::Mat& GridMatrix, int row, int col, std::vector<cv::Point2f>& ret);
void getDestinationPoints(const cv::Mat& GridMatrix, int row, int col, std::vector<cv::Point2f>& ret, double zoom = 1.);
void transformBlock(const cv::Mat& src_img, const cv::Mat& dst_img, std::vector<cv::Point2f> from, std::vector<cv::Point2f> to, const cv::Mat& aperture);
void LFRefocus(const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& GridMatrix, int rows, int cols, double zoom, double apertureSize, double offsetX, double offsetY);
void RenderingTask(cv::Mat* src_img);

volatile bool isOk = true;
static std::mutex m;

int main(int argc, char** argv)
{
	std::cout << argc << "\n";
	std::string calib_path = "D:/workspace/LFCameraCalib/camera_calibration.yaml";

	cv::FileStorage calib_config(calib_path, cv::FileStorage::READ);
	cv::Mat GridMatrix;
	int GridRows(calib_config["GridRows"]), GridCols(calib_config["GridCols"]);
	calib_config["GridMatrix"] >> GridMatrix;

	cv::Mat frameMat(3288, 4384, CV_8UC1), frameMatBGR(3288, 4384, CV_8UC3), src_img, rendering_image;
	MVCFRAMEINFO Frame;
	std::thread renderingThread;
	std::cout << argc << "\n";
	if (argc > 1)
	{
		m.lock();
		std::cout << "Load " << argv[1] << "\n";
		frameMatBGR = cv::imread(argv[1]);
		m.unlock();
		RenderingTask(&frameMatBGR);
		goto FINISH123;
	}
	renderingThread = std::thread(&RenderingTask, &frameMatBGR);
	int devNo(0), devQty;
	devQty = MVC_GetDeviceNumber();
	std::cout << "Device quantity: " << devQty << "\n";
	if (devQty <= 0)
	{
		std::cerr << "No device online\n";
		exit(1);
	}
	MVC_AutoIPConfig(devNo);
	MVC_SetParameter(devNo, MVCDISP_ISDISP, 0);
	MVC_SetNetPacketSize(devNo, 1440);

	MVC_OpenDevice(devNo);
	MVC_EnableCapture(devNo);

	int key = 0, mode = 'e';
	DWORD gain = 90, exposure = 16000;

	MVC_SetParameter(0, MVCADJ_INTTIME, exposure);
	MVC_SetParameter(0, MVCADJ_DACGAIN, gain);

	while (isOk)
	{
		MVC_GetRawData(devNo, &Frame);
		//std::cout << std::setw(10) << std::left << Frame.FRAMEID << std::setw(4) << ": [" << Frame.Width << ", " << Frame.Height << "] " << Frame.lBufSize << "\n";
		//std::cout << "Pixel type: " << Frame.PixelID << "\n";
		frameMat.data = Frame.lBufPtr;
		m.lock();
		if (isOk)
			cv::cvtColor(frameMat, frameMatBGR, cv::COLOR_BayerGB2BGR);
		m.unlock();
		//cv::cvtColor(frameMatBGR, src_img, cv::COLOR_BGR2GRAY);
		cv::pyrDown(frameMatBGR, src_img);
		cv::imshow("src_img", src_img);
		key = cv::waitKey(1);

		if (key == 'e' || key == 'g')
		{
			mode = key;
		}
		else if (key == ',')
		{
			if (mode == 'e')
			{
				exposure = std::max(0ul, exposure - 1000ul);
				MVC_SetParameter(0, MVCADJ_INTTIME, exposure);
				std::cout << "Exposure: " << exposure << "\n";
			}
			else if (mode == 'g')
			{
				gain = std::max(0ul, gain - 5u);
				MVC_SetParameter(0, MVCADJ_DACGAIN, gain);
				std::cout << "Gain: " << gain << "\n";
			}
		}
		else if (key == '.')
		{
			if (mode == 'e')
			{
				exposure = std::min(40000ul, exposure + 2000ul);
				MVC_SetParameter(0, MVCADJ_INTTIME, exposure);
				std::cout << "Exposure: " << exposure << "\n";
			}
			else if (mode == 'g')
			{
				gain = std::min(120ul, gain + 5ul);
				MVC_SetParameter(0, MVCADJ_DACGAIN, gain);
				std::cout << "Gain: " << gain << "\n";
			}
		}
		else if (key == 'b')
		{
			std::cout << "Auto white balance " << MVC_AutoWhiteBalance(0ul) << "\n";


		}
	}

	MVC_DisableCapture(devNo);
	MVC_CloseDevice(devNo);

	renderingThread.join();

FINISH123:;
	std::cout << "Finish\n";
}

void getSourcePoints(const cv::Mat& GridMatrix, int row, int col, std::vector<cv::Point2f>& ret)
{
	double dx[4]{ -0.5, -0.5, 0.5, 0.5 }, dy[4]{ -0.5, 0.5, 0.5, -0.5 };

	for (int i = 0; i < 4; ++i)
	{
		cv::Mat v = (cv::Mat_<double>(3, 1) << 1, col + dx[i], row + dy[i]);
		cv::Mat r = (GridMatrix * v);
		ret.push_back(cv::Point2d(r.at<double>(0), r.at<double>(1)));
	}
}

void getDestinationPoints(const cv::Mat& GridMatrix, int row, int col, std::vector<cv::Point2f>& ret, double zoom)
{
	double dx[4]{ -1.1, -1.1, 1.1, 1.1 }, dy[4]{ -1.1, 1.1, 1.1, -1.1 };

	for (int i = 0; i < 4; ++i)
	{
		cv::Mat v = (cv::Mat_<double>(3, 1) << 0, 0.4 * col + dx[i] * zoom, 0.4 * row + dy[i] * zoom);
		cv::Mat r = (GridMatrix * v);
		ret.push_back(cv::Point2d(r.at<double>(0), r.at<double>(1)));
	}
}

void transformBlock(const cv::Mat& src_img, cv::Mat& dst_img, std::vector<cv::Point2f> from, std::vector<cv::Point2f> to, const cv::Mat& aperture)
{
	if (src_img.empty())
		throw std::exception("src_img empty!");
	cv::Rect rect = cv::boundingRect(to);
	cv::Mat block = cv::Mat::zeros(rect.height, rect.width, src_img.type()), block_ = cv::Mat::zeros(rect.height, rect.width, src_img.type());



	std::vector<cv::Point2f> tmp_to;
	for (auto& i : to)
	{
		tmp_to.push_back(i - static_cast<cv::Point2f>(rect.tl()));
	}
	cv::Mat transMat = cv::getPerspectiveTransform(from, tmp_to);
	cv::warpPerspective(src_img, block, transMat, block.size());
	block &= aperture;
	int shiftX(0), shiftY(0);
	if (rect.x < 0)
		shiftX = rect.x;
	else if (rect.x + rect.width > dst_img.cols)
		shiftX = (dst_img.cols - rect.x - rect.width);
	if (rect.y < 0)
		shiftY = rect.y;
	else if (rect.y + rect.height > dst_img.rows)
		shiftY = (dst_img.rows - rect.y - rect.height);

	cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, shiftX, 0, 1, shiftY);
	cv::warpAffine(block, block_, M, block.size(), 1, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	rect.x = std::max(rect.x, 0);
	rect.x = std::min(rect.x, dst_img.cols - rect.width);
	rect.y = std::max(rect.y, 0);
	rect.y = std::min(rect.y, dst_img.rows - rect.height);

	dst_img(rect) += block_;
}

void transformBlockMask(cv::Mat& dst_img, std::vector<cv::Point2f> from, std::vector<cv::Point2f> to, const cv::Mat& aperture)
{

	cv::Rect rect = cv::boundingRect(to);
	cv::Mat aperture_ = cv::Mat::zeros(rect.size(), CV_8UC1);
	//cv::Mat aperture = cv::getStructuringElement(cv::MORPH_ELLIPSE, rect.size()), aperture_=cv::Mat::zeros(rect.size(), CV_8UC1);
	//cv::Size sz = aperture.size();
	//sz.width *= (1. - apertureSize);
	//sz.height *= (1. - apertureSize);
	//sz.width += (1 - sz.width % 2);
	//sz.height += (1 - sz.height % 2);
	//cv::erode(aperture, aperture, cv::getStructuringElement(cv::MORPH_ELLIPSE, sz), cv::Point(-1, -1), 1, 0, cv::Scalar(0, 0, 0));
	//cv::cvtColor(aperture, aperture, cv::COLOR_GRAY2BGR);

	int shiftX(0), shiftY(0);
	if (rect.x < 0)
		shiftX = rect.x;
	else if (rect.x + rect.width > dst_img.cols)
		shiftX = (dst_img.cols - rect.x - rect.width);
	if (rect.y < 0)
		shiftY = rect.y;
	else if (rect.y + rect.height > dst_img.rows)
		shiftY = (dst_img.rows - rect.y - rect.height);

	cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, shiftX, 0, 1, shiftY);
	cv::warpAffine(aperture, aperture_, M, aperture_.size(), 1, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
	rect.x = std::max(rect.x, 0);
	rect.x = std::min(rect.x, dst_img.cols - rect.width);
	rect.y = std::max(rect.y, 0);
	rect.y = std::min(rect.y, dst_img.rows - rect.height);

	cv::cvtColor(aperture_, aperture_, cv::COLOR_BGR2GRAY);
	aperture_.convertTo(aperture_, CV_32F);
	cv::GaussianBlur(aperture_, aperture_, cv::Size(5, 5), 0.7);
	//cv::blur(aperture_, aperture_, cv::Size(3, 3));
	dst_img(rect) += aperture_;
}

void getAperture(cv::Mat& aperture, const std::vector<cv::Point2f>& to, double apertureSize, double offsetX, double offsetY)
{

	cv::Rect rect = cv::boundingRect(to);

	aperture = cv::getStructuringElement(cv::MORPH_ELLIPSE, rect.size());
	cv::Size sz = rect.size();
	sz.width *= (1. - apertureSize);
	sz.height *= (1. - apertureSize);
	sz.width += (1 - sz.width % 2);
	sz.height += (1 - sz.height % 2);
	cv::erode(aperture, aperture, cv::getStructuringElement(cv::MORPH_ELLIPSE, sz), cv::Point(-1, -1), 1, 0, cv::Scalar(0, 0, 0));

	cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, offsetX / 2. * sz.width, 0, 1, offsetY / 2. * sz.height);
	cv::warpAffine(aperture, aperture, M, aperture.size(), 1, 0, cv::Scalar(0));

	cv::cvtColor(aperture, aperture, cv::COLOR_GRAY2BGR);
}

void LFRefocus(const cv::Mat& src_img, cv::Mat& dst_img, const cv::Mat& GridMatrix, int rows, int cols, double zoom, double apertureSize, double offsetX, double offsetY)
{

	dst_img = cv::Mat::zeros(src_img.rows * 0.4, src_img.cols * 0.4, CV_32FC3);
	cv::Mat aperture;
	cv::Mat dst_img_mask;
	dst_img_mask = cv::Mat::zeros(src_img.rows * 0.4, src_img.cols * 0.4, CV_32F);
	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			//std::cout << "RF" << i << "\t" << j << "\n";
			if (((i + j) & 0x01) == 0x00)
				continue;
			std::vector<cv::Point2f> from, to;
			getSourcePoints(GridMatrix, i, j, from);
			getDestinationPoints(GridMatrix, i, j, to, zoom);
			cv::Rect rect = cv::boundingRect(to);
			if (((i == 0) && (j == 0)) || (rect.size() != aperture.size()))
			{
				getAperture(aperture, to, apertureSize, offsetX, offsetY);
			}
			transformBlock(src_img, dst_img, from, to, aperture * 255);
			transformBlockMask(dst_img_mask, from, to, aperture);
		}
	}
	//cv::imshow("Block Mask", aperture * 255);
	cv::Mat tmpMat = ((dst_img_mask <= 0.001) / 255);
	tmpMat.convertTo(tmpMat, CV_32F);
	dst_img_mask += tmpMat;
	cv::merge(std::vector<cv::Mat>{dst_img_mask, dst_img_mask, dst_img_mask}, dst_img_mask);
	//cv::cvtColor(dst_img_mask, dst_img_mask, cv::COLOR_GRAY2BGR);
	//std::cout << dst_img_mask << "\n";
	//cv::imshow("Mask", dst_img_mask*10);
	//dst_img_mask.convertTo(dst_img_mask, CV_32FC3);
	dst_img /= dst_img_mask;
	dst_img.convertTo(dst_img, CV_8UC3);
	cv::medianBlur(dst_img, dst_img, 7);
}

void RenderingTask(cv::Mat* src_img)
{
	std::string calib_path = "D:/workspace/LF/LFCameraCalib/camera_calibration.yaml";

	cv::FileStorage calib_config(calib_path, cv::FileStorage::READ);
	cv::Mat GridMatrix;
	int GridRows(calib_config["GridRows"]), GridCols(calib_config["GridCols"]);	
	calib_config["GridMatrix"] >> GridMatrix;

	double global_zoom = 0.43, global_aperture = 0.83, offsetX(0.), offsetY(0.);

	GridCols = 35;
	GridRows = 25;

	GridMatrix = ((cv::Mat_<double>(2, 3) <<
		147, 118.07692308, 0.23809524,
		167, -0.38461538, 117.38095238));
	GridMatrix.col(1) *= 0.997;
	GridMatrix.col(2) *= 1.001;

	cv::Mat src_img_cp, rendering_image;
	cv::Mat leftAperture, rightAperture;
	while (isOk)
	{
		if (!src_img->empty())
		{
			m.lock();
			src_img->copyTo(src_img_cp);
			m.unlock();
			std::cout << global_aperture << "\t" << global_zoom << "\t" << offsetX << "\t" << offsetY << "\n";
			LFRefocus(src_img_cp, rendering_image, GridMatrix, GridRows, GridCols, global_zoom, global_aperture, offsetX, offsetY);
			cv::imshow("RF", rendering_image);
			//LFRefocus(src_img_cp, leftAperture, GridMatrix, GridRows, GridCols, global_zoom, global_aperture, -1, offsetY);
			//LFRefocus(src_img_cp, rightAperture, GridMatrix, GridRows, GridCols, global_zoom, global_aperture, 1, offsetY);
			//cv::imshow("LEFT", leftAperture);
			//cv::imshow("RIGHT", rightAperture);
		}
		int key = cv::waitKeyEx(1);
		if (key == 27)
			break;
		if (key == 'z')
		{
			if (!rendering_image.empty())
			{
				cv::imwrite("rf.bmp", rendering_image);
			}
		}
		else if (key == 'q')
		{
			global_zoom = std::max(0.0, global_zoom - 0.01);
		}
		else if (key == 'w')
		{
			global_zoom = std::min(1., global_zoom + 0.01);
		}
		else if (key == 'a')
		{
			global_aperture = std::max(0.0, global_aperture - 0.01);
		}
		else if (key == 's')
		{
			global_aperture = std::min(1., global_aperture + 0.01);
		}
		else if (key == 0x00250000)
		{
			offsetX = std::max(-1., offsetX - 0.1);
		}
		else if (key == 0x00260000)
		{

			offsetY = std::max(-1., offsetY - 0.1);
		}
		else if (key == 0x00270000)
		{

			offsetX = std::min(1., offsetX + 0.1);
		}
		else if (key == 0x00280000)
		{

			offsetY = std::min(1., offsetY + 0.1);
		}
	}
	//src_img->copyTo(src_img_cp);
	//for (int i = 0; i < 20; ++i)
	//{
	//	for (int j = 0; j < 20; ++j)
	//	{
	//		std::cout << i << "\t" << j << "\n";
	//		LFRefocus(src_img_cp, rendering_image, GridMatrix, GridRows, GridCols, global_zoom, global_aperture, static_cast<double>(j) / 10. - 1., static_cast<double>(i) / 10. - 1.);
	//		char nm[20]{ 0 };
	//		sprintf(nm, "%02d_%02d.bmp", i, j);
	//		cv::imwrite(nm, rendering_image);
	//	}
	//}
	m.lock();
	isOk = false;
	m.unlock();
}