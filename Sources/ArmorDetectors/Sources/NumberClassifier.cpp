#include <LangYa/ArmorDetectors/NumberClassifier.hpp>
#include <spdlog/spdlog.h>
#include <opencv2/opencv.hpp>

namespace LangYa:: inline ArmorDetectors
{
	float GetDistance(const cv::Point2f& a, const cv::Point2f& b)
	{
		const auto point_diff = a - b;
		return std::sqrt(point_diff.dot(point_diff));
	}

	// 加载模型
	bool NumberClassifier::LoadModel(const std::string& model_path)
	{
		spdlog::trace("NumberClassifier> loading svm model at path({})", model_path);

		SVMModel = cv::ml::SVM::load(model_path);
		if (SVMModel.empty())
		{
			spdlog::error("NumberClassifier> cannot load svm model");
			return false;
		}
		return true;
	}

	bool AffineNumber(const cv::Mat& frame, const std::vector<cv::Point2f>& corners, cv::Mat& numberROI)
	{
		static float classify_width_ratio = 0.2f;
		static float classify_height_ratio = 0.5f;

		// 求解完全包围这个框的最大矩形
		cv::Point2f correct_points[4];
		cv::Point2f width_vec = (corners[1] - corners[0] + corners[2] - corners[3]) / 2;
		cv::Point2f height_vec = (corners[3] - corners[0] + corners[2] - corners[1]) / 2;

		correct_points[0] = corners[0] + classify_width_ratio * width_vec - classify_height_ratio * height_vec;
		correct_points[1] = corners[1] - classify_width_ratio * width_vec - classify_height_ratio * height_vec;
		correct_points[2] = corners[2] - classify_width_ratio * width_vec + classify_height_ratio * height_vec;
		correct_points[3] = corners[3] + classify_width_ratio * width_vec + classify_height_ratio * height_vec;

		int width = GetDistance(correct_points[0], correct_points[1]);
		int height = GetDistance(correct_points[1], correct_points[2]);
		auto min_point = cv::Point2f(9999.0f, 9999.0f);
		auto max_point = cv::Point2f(0.0f, 0.0f);
		for (int i = 0; i < 4; i++)
		{
			min_point.x = min_point.x < correct_points[i].x ? min_point.x : correct_points[i].x;
			min_point.y = min_point.y < correct_points[i].y ? min_point.y : correct_points[i].y;
			max_point.x = max_point.x > correct_points[i].x ? max_point.x : correct_points[i].x;
			max_point.y = max_point.y > correct_points[i].y ? max_point.y : correct_points[i].y;
		}
		min_point.x = MAX(min_point.x, 0);
		min_point.y = MAX(min_point.y, 0);
		max_point.x = MIN(max_point.x, frame.cols);
		max_point.y = MIN(max_point.y, frame.rows);

		// 截取
		numberROI = frame(cv::Rect(min_point, max_point));

		for (int i = 0; i < 4; i++)
		{
			correct_points[i] -= min_point;
		}

		// 制作重映射对应点
		cv::Point2f remap_points[4];
		remap_points[0] = cv::Point2f(0, 0);
		remap_points[1] = cv::Point2f(width, 0);
		remap_points[2] = cv::Point2f(width, height);
		remap_points[3] = cv::Point2f(0, height);

		// 进行重映射
		cv::Mat trans_matrix = getPerspectiveTransform(correct_points, remap_points);
		cv::Mat output_roi;
		output_roi.create(cv::Size(width, height), CV_8UC3);

		if (numberROI.empty() || output_roi.empty())
		{
			return false;
		}
		warpPerspective(numberROI, output_roi, trans_matrix, output_roi.size());

		// //从重映射中取得目标图像
		resize(output_roi, numberROI, cv::Size(32, 32)); // 根据训练的数据大小来判断大小
		return true;
	}

	void AutoGamma(const cv::Mat& img, cv::Mat& out)
	{
		const int channels = img.channels();

		auto mean = cv::mean(img); // 均值
		mean[0] = std::log10(0.4) / std::log10(mean[0] / 255);

		if (channels == 3) // 3channels
		{
			mean[1] = std::log10(0.4) / std::log10(mean[1] / 255);
			mean[2] = std::log10(0.4) / std::log10(mean[2] / 255);

			float mean_end = 0;
			for (int i = 0; i < 3; ++i)
				mean_end += mean[i];

			mean_end /= 3.0f;
			for (int i = 0; i < 3; ++i)
				mean[i] = mean_end;
		}
		/*gamma_table*/
		cv::Mat lut(1, 256, img.type());

		if (channels == 1)
		{
			for (int i = 0; i < 256; i++)
			{
				/*[0,1]*/
				float Y = i * 1.0f / 255.0;
				Y = std::pow(Y, mean[0]);
				lut.at<unsigned char>(0, i) = cv::saturate_cast<unsigned char>(Y * 255);
			}
		}
		else
		{
			for (int i = 0; i < 256; ++i)
			{
				float Y = i * 1.0f / 255.0;
				auto B = cv::saturate_cast<unsigned char>(std::pow(Y, mean[0]) * 255);
				auto G = cv::saturate_cast<unsigned char>(std::pow(Y, mean[1]) * 255);
				auto R = cv::saturate_cast<unsigned char>(std::pow(Y, mean[2]) * 255);

				lut.at<cv::Vec3b>(0, i) = cv::Vec3b(B, G, R);
			}
		}
		LUT(img, lut, out);
	}

	cv::Mat pixel(const cv::Mat& image, double gamma, double mu, double sigma, int flag)
	{
		double K;
		cv::Mat img;
		image.copyTo(img);
		int rows = image.rows;
		int cols = image.cols;
		int channels = image.channels();
		if (flag)
		{
			for (int i = 0; i < cols; ++i)
			{
				for (int j = 0; j < rows; ++j)
				{
					if (channels == 3)
						for (int k = 0; k < channels; ++k)
						{
							float pix = static_cast<float>(img.at<cv::Vec3b>(i, j)[k]) / 255.0;
							img.at<cv::Vec3b>(i, j)[k] = cv::saturate_cast<uchar>(std::pow(pix, gamma) * 255.0f);
						}
					else
					{
						float pix = static_cast<float>(img.at<uchar>(i, j)) / 255.0f;
						img.at<uchar>(i, j) = cv::saturate_cast<uchar>(std::pow(pix, gamma) * 255.0f);
					}
				}
			}
		}
		else
		{
			for (int i = 0; i < cols; ++i)
			{
				for (int j = 0; j < rows; ++j)
				{
					if (channels == 3)
						for (int k = 0; k < channels; ++k)
						{
							float pix = static_cast<float>(img.at<cv::Vec3b>(i, j)[k]) / 255.0;
							double t = std::pow(mu, gamma);
							K = std::pow(pix, gamma) + (1 - std::pow(pix, gamma)) * t;
							img.at<cv::Vec3b>(i, j)[k] = cv::saturate_cast<uchar>(std::pow(pix, gamma) / K * 255.0f);
						}
					else
					{
						float pix = static_cast<float>(img.at<uchar>(i, j)) / 255.0;
						double t = std::pow(mu, gamma);
						K = std::pow(pix, gamma) + (1 - std::pow(pix, gamma)) * t;
						img.at<uchar>(i, j) = cv::saturate_cast<uchar>(std::pow(pix, gamma) / K * 255.0f);
					}
				}
			}
		}
		return img;
	}


	cv::Mat AutoGammaCorrect(const cv::Mat& image)
	{
		cv::Mat img;
		cv::Mat dst;
		image.copyTo(dst);
		image.copyTo(img);

		cvtColor(img, img, cv::COLOR_BGR2HSV);
		cvtColor(dst, dst, cv::COLOR_BGR2GRAY);
		const int type = img.type();
		// assert(type == CV_8UC1 || type == CV_8UC3);

		std::vector<cv::Mat> channels(3);
		split(img, channels);
		cv::Mat V = channels[2];
		V.convertTo(V, CV_32F);
		double max_v;
		minMaxIdx(V, nullptr, &max_v);
		V /= 255.0;

		cv::Mat Mean, Sigma;
		meanStdDev(V, Mean, Sigma);

		double mu, sigma; /*均值and方差*/
		bool High_contrast = false;
		bool High_bright = false;
		mu = Mean.at<double>(0);
		sigma = Sigma.at<double>(0);
		// std::cout<<"mu = "<<mu<<"sigma = "<<sigma<<std::endl;

		if (4 * sigma > 0.3333)
		{
			High_contrast = true;
			//    std::cout<<"High_con"<<std::endl;
		}
		if (mu > 0.65)
		{
			High_bright = true;
			//    std::cout<<"High_bri"<<std::endl;
		}
		double gamma;

		if (High_contrast)
		{
			gamma = std::exp((1 - (mu + sigma)) / 2.0);
		}
		else
		{
			gamma = -std::log(sigma) / std::log(2);
		}
		// std::cout << gamma << std::endl;

		return pixel(dst, gamma, mu, sigma, High_bright);
	}

	int NumberClassifier::Predict(const cv::Mat& frame, const std::vector<cv::Point2f>& corners) const
	{
		cv::Mat number_roi{};
		if (!AffineNumber(frame, corners, number_roi) || number_roi.empty())
		{
			return -1;
		}

		number_roi = AutoGammaCorrect(number_roi);
		std::vector<float> hog_descriptors;
		// 对图片提取hog描述子存在hog_descriptors中，hog描述子是向量，不是矩阵
		HOG.compute(number_roi, hog_descriptors, cv::Size{8, 8}, cv::Size(0, 0));
		const int size = static_cast<int>(hog_descriptors.size());
		cv::Mat descriptors_mat(1, size, CV_32FC1); // 行向量
		for (int i = 0; i < hog_descriptors.size(); ++i)
		{
			descriptors_mat.at<float>(0, i) = hog_descriptors[i] * 100;
		}

		// 把提取的本张图片的hog描述子放进svm预测器中进行预测
		cv::Mat result{};
		(void)SVMModel->predict(descriptors_mat, result);
		return static_cast<int>(result.at<float>(0));
	}
}
