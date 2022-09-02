#include "steger_process.h"
#include "shape/fit.h"
#include "shape/shape.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
namespace image {
StegerProcess::StegerProcess(std::string name, double sigma)
	:ImageProcessBase(name),
	callback_lock_(),
	callback_on_points_(),
	steger_(),
	steger_sigma_(sigma),
	profile_model_(),
	type_(common::WeldType::UNKNOWN),
	lastROI_()
	{}

void StegerProcess::store() const {
	ImageProcessBase::store();
}
void StegerProcess::process() {
	std::lock_guard<std::mutex> guard(value_lock_);
	cv::Mat roi_image = origin_(roi_).clone();
	lastROI_ = roi_;
	if (roi_image.cols == 0 || roi_image.rows == 0) {
		lastIsBad_ = true;
		LOG(WARNING) << "[StegerProcess::process] Image has a zero size";
		return;
	}
	//std::cout << roi_image.size() << std::endl;
	//cv::Mat roi_to_display = roi_image.clone();
	steger_.reset(new image::StergerLineDetector());
	steger_->Compute(roi_image, steger_sigma_);

	if (callback_on_pointcloud_) {
		std::lock_guard<std::mutex> hold(callback_lock_);
		lastPoints_ = steger_->GetSubPixelVec();
		for (auto &p : lastPoints_) {
			const auto true_x = lastROI_.x + p.y, true_y = lastROI_.y + p.x;
			p.x = true_x; p.y = true_y;
		}
		LOG(INFO) << "[StegerProcess::process] PointCloud Callback called, timestamp: " << timestamp_;
		callback_on_pointcloud_(lastPoints_, timestamp_);
	}

	lastIsBad_ = false;
	// ROI may be modified on the fly, store last used roi is safer for later usage(like recovery of image x, y)

}
void StegerProcess::output() {
	_display(origin_, kOrigin);
	_display(origin_(roi_).clone(), kROI);
	_display(steger_->GetFlagMat(), kROI2);
}
void StegerProcess::registerPointsCallback(const PointsCallback &f) {
	std::lock_guard<std::mutex> hold(callback_lock_);
	callback_on_points_ = f;
}

void StegerProcess::registerPointCloudCallback(const PointCloudCallback &f) {
	std::lock_guard<std::mutex> hold(callback_lock_);
	callback_on_pointcloud_ = f;
}

void StegerProcess::setProfileModel(shape::ProfileModelPtr model_p) {
	std::lock_guard<std::mutex> hold(callback_lock_);
	profile_model_ = model_p;
}
void StegerProcess::setStegerSigma(double sigma) {
	std::lock_guard<std::mutex> hold(value_lock_);
	steger_sigma_ = sigma;
}
shape::ProfileModelPtr StegerProcess::getProfileModel() const{
	std::lock_guard<std::mutex> hold(callback_lock_);
	return profile_model_;
}
StegerProcessV::StegerProcessV(std::string name, double sigma) :
	StegerProcess(name, sigma) {
	type_ = common::WeldType::V;
}

void StegerProcessV::process() {
	StegerProcess::process();
	if (!lastIsBad_) {
		std::lock_guard<std::mutex> hold(callback_lock_);
		callback_on_pointcloud_(steger_->GetSubPixelVec(), timestamp_);
		auto coefs_of_models = shape::FitProfileModel(steger_->GetSubPixelVec(), profile_model_, true);

		// FOR DEBUG
		/*
		LOG(INFO) << "[Callback-emit_fitted_points] Fitting..";
		std::ostringstream model_repr;
		for (auto coef : coefs_of_models) {
			std::ostringstream oss;
			if (!coef.empty()){
				// Convert all but the last element to avoid a trailing ","
				std::copy(coef.begin(), coef.end() - 1,
					std::ostream_iterator<float>(oss, ","));
				// Now add the last element with no delimiter
				oss << coef.back();
			}
			model_repr << "[" << oss.str() <<"]";
		}
		LOG(INFO) << "[Callback-emit_fitted_points]" << "Fitted coef of V:" << model_repr.str();
		*/
		const auto before_end = coefs_of_models.cend() - 1;
		std::vector<shape::OutputVec> points;
		for (auto i = coefs_of_models.cbegin(); i != before_end; ++i) {
			auto point = shape::IntersectOfTwoLines(*i, *(i + 1));
			bool success = point.size() == 2;
			if (success) {
				auto true_x = point[1] + lastROI_.x, true_y = point[0] + lastROI_.y;
				point[0] = true_x;
				point[1] = true_y;
			}
			points.emplace_back(success, point);
		}
		if (callback_on_points_) {
			callback_on_points_(points, timestamp_);
		}
		if (points.size() == 3 && std::get<0>(points[1])) {
			cv::Mat to_plot;
			cv::cvtColor(origin_, to_plot, CV_GRAY2RGB);
			const auto xy = std::get<1>(points[1]);
			auto row = int(xy[1]), col = int(xy[0]);
			auto center = cv::Point(col, row);
			DrawCross(to_plot, center, cv::Scalar(255,0,0), 5, 40);
			//cv::line(to_plot, center, 
			// display;
			_display(to_plot, kProfile);
		}
		else {
			LOG(INFO) << "StegerV: invalid point output";
		}

	}
	else {
		LOG(INFO) << "StegerV: last is bad. timestamp: " <<timestamp_;
	}
}
void StegerProcessV::output() {
	if (counter_ % 5 == 0) {
		_display(origin_, kOrigin);
		_display(steger_->GetFlagMat(), kROI2);
	}
}
StegerProcessT::StegerProcessT(std::string name, double sigma) :
	StegerProcess(name, sigma) {
	type_ = common::WeldType::CORNER;
}
void StegerProcessT::process() {
	StegerProcess::process();
	if (!lastIsBad_) {
		std::lock_guard<std::mutex> hold(callback_lock_);
		auto coefs_of_models = shape::FitProfileModel(steger_->GetSubPixelVec(), profile_model_, true);
		// FOR DEBUG
		/*
		LOG(INFO) << "[Callback-emit_fitted_points] Fitting..";
		std::ostringstream model_repr;
		for (auto coef : coefs_of_models) {
		std::ostringstream oss;
		if (!coef.empty()){
		// Convert all but the last element to avoid a trailing ","
		std::copy(coef.begin(), coef.end() - 1,
		std::ostream_iterator<float>(oss, ","));
		// Now add the last element with no delimiter
		oss << coef.back();
		}
		model_repr << "[" << oss.str() <<"]";
		}
		LOG(INFO) << "[Callback-emit_fitted_points]" << "Fitted coef of V:" << model_repr.str();
		*/
		const auto before_end = coefs_of_models.cend() - 1;
		std::vector<shape::OutputVec> points;
		{
			std::lock_guard<std::mutex> guard(value_lock_);
			for (auto i = coefs_of_models.cbegin(); i != before_end; ++i) {
				auto point = shape::IntersectOfTwoLines(*i, *(i + 1));
				bool success = point.size() == 2;
				if (success) {
					auto true_x = point[1] + lastROI_.x, true_y = point[0] + lastROI_.y;
					point[0] = true_x;
					point[1] = true_y;
				}
				points.emplace_back(success, point);
			}
		}
		if (callback_on_points_) {
			callback_on_points_(points, timestamp_);
		}
		if (points.size() == 1 && std::get<0>(points[0])) {
			cv::Mat to_plot;
			cv::cvtColor(origin_, to_plot, CV_GRAY2RGB);
			auto xy = std::get<1>(points[0]);
			auto row = int(xy[1]), col = int(xy[0]);
			auto center = cv::Point(col, row);
			DrawCross(to_plot, center, cv::Scalar(0, 0, 255), 5, 40);
			const auto sz_should_be_2 = profile_model_->GeModelSize();
			if (sz_should_be_2 == 2) {
				const size_t x_r = to_plot.cols;
				// the x,y of model and steger algo output is reversed from the true x,y...
				const auto & coef_1 = coefs_of_models[0], coef_2 = coefs_of_models[1];
				size_t y11, y12, y21, y22;
				std::tie(y11, y12, std::ignore, std::ignore) = profile_model_->GetModelNo(0);
				std::tie(y21, y22, std::ignore, std::ignore) = profile_model_->GetModelNo(1);
				cv::line(to_plot, cv::Point(0, y11), cv::Point(x_r, y11), cv::Scalar(0, 0, 200), 2);
				cv::line(to_plot, cv::Point(0, y12), cv::Point(x_r, y12), cv::Scalar(0, 0, 100), 2);
				cv::line(to_plot, cv::Point(0, y21), cv::Point(x_r, y21), cv::Scalar(0, 0, 200), 2);
				cv::line(to_plot, cv::Point(0, y22), cv::Point(x_r, y22), cv::Scalar(0, 0, 100), 2);
				/*
				if (coef_1.size() >= 6 && coef_1[3] != 0.f) {
					const auto x1 = double(coef_1[1]) + (double(y11) - double(coef_1[0])) * double(coef_1[4]) / double(coef_1[3]);
					//const auto x1 = coef_1[3] / coef_1[4] * y11;
					//cv::line(to_plot()
					cv::line(to_plot, center, cv::Point(x1 + lastROI_.x, y11 + lastROI_.y), cv::Scalar(0, 255, 0), 3);
				}
				if (coef_2.size() >= 6 && coef_2[3] != 0.f) {
					const auto x2 = double(coef_2[1]) + (double(y11) - double(coef_2[0])) * double(coef_2[4]) / double(coef_2[3]);
					//const auto x2 = coef_2[3] / coef_2[4] * y22;
					cv::line(to_plot, center, cv::Point(x2 + lastROI_.x, y22 + lastROI_.y), cv::Scalar(0, 150, 0), 3);
				}
				*/
			}
			//profile_model_->
			// display;
			_display(to_plot(roi_).clone(), kROI);
			_display(steger_->GetFlagMat(), kROI2);
		}
		else {
			LOG(INFO) << "StegerT: invalid point output";
			_display(origin_(roi_).clone(), kROI);
			_display(steger_->GetFlagMat(), kROI2);
		}
	}
	else {
		LOG(INFO) << "StegerT: last is bad. timestamp: " << timestamp_;
	}
}
void StegerProcessT::output() {
	_display(origin_, kOrigin);
}
StegerProcessButt::StegerProcessButt(std::string name, double sigma) :
	StegerProcess(name, sigma) {
	type_ = common::WeldType::BUTT;
}
void StegerProcessButt::process() {
	StegerProcess::process();
	std::lock_guard<std::mutex> hold(callback_lock_);
	LOG(INFO) << "StegerB: invalid point output";
	lastROI_ = roi_;
	cv::Mat roi_image = origin_(roi_).clone();
	
	auto m_subpixel = steger_->GetSubPixelVec();

	cv::Mat m_lsd= cv::Mat::zeros(roi_image.size(), CV_8UC1);
	std::vector<cv::Point> m_CPoints;

	for (auto j = m_subpixel.begin(); j != m_subpixel.end(); ++j) {		
		if ((*j).angle > 75 && (*j).angle < 105) {
			m_lsd.at<uchar>((*j).x, (*j).y) = 255;
			if ((*j).x > 0 && (*j).x < 120) {
				m_CPoints.push_back(cv::Point((*j).y, (*j).x));
			}
			else if ((*j).x > 250 && (*j).x < 400) {
				m_CPoints.push_back(cv::Point((*j).y, (*j).x));
			}
		}
	}

	cv::Vec4f line_para1;
	cv::fitLine(m_CPoints, line_para1, cv::DIST_L1, 0, 1e-5, 1e-5);
	cv::Point point1, point2;
	cv::Point point0;
	double k = line_para1[1] / line_para1[0];
	point0.x = line_para1[2];
	point0.y = line_para1[3];

	cv::Mat aaa_2 = roi_image.clone();
	cv::Point x_1(0, 0), x_2(0, 0);
	cv::Mat k4;
	cv::Mat kern4 = (cv::Mat_<float>(5, 5) <<
		1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0,
		1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0,
		1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0,
		1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0,
		1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0, 1 / 25.0);

	cv::filter2D(roi_image, k4, CV_32F, kern4);
	point1.y = 0;
	point1.x = (point1.y - point0.y) / k + point0.x;
	auto m_1 = k4.at<float>(point1);
	int _up = 0;
	int  _down=0,m_down = 0;
	bool downflag=true,upflag=0,x_1flag=0,x_2flag=0;
	//float m_1;
	for (int i = 1; i < lastROI_.height - 50; i++) {
		point1.y = i;
		point1.x = (point1.y - point0.y) / k + point0.x;
		auto m = k4.at<float>(point1);
		//std::cout << m << "\t" << point1.y << "\t" << point1.x << std::endl;
		if (m < 100 && m_1>100 && downflag) {
			downflag = false;
			x_1 = point1;
		}
		else if (m > 100 && m_1 < 100) {
			x_2 = point1;
		}
		m_1 = m;
	}
	std::vector<shape::OutputVec> points;
	shape::VecXf point(2);
	point[0] = 0.5*(x_1.x + x_2.x) + lastROI_.x;
	point[1] = 0.5*(x_1.y + x_2.y) + lastROI_.y;
	points.emplace_back(true, point);
	LOG(INFO) << "StegerB:  timestamp: " << timestamp_<<"x: "<< point[0]<<"y: " << point[1];
	if (callback_on_points_) {
		callback_on_points_(points, timestamp_);
	}
	cv::Mat to_plot;
	cv::cvtColor(roi_image, to_plot, CV_GRAY2RGB);

	cv::circle(to_plot, cv::Point(0.5*(x_1.x + x_2.x),0.5*(x_1.y + x_2.y)), 2, cv::Scalar(255, 0, 0), 2);
	cv::circle(to_plot, x_1, 2, cv::Scalar(0, 0, 255), 2);
	cv::circle(to_plot, x_2, 2, cv::Scalar(0,  255,0), 2);

	_display(m_lsd, kROI2);
	_display(to_plot, kProfile);
	_display(origin_(roi_).clone(), kROI);
}
void StegerProcessButt::output() {
	_display(origin_, kOrigin);
}
StegerProcessPtr MakeImageProcessSteger(common::WeldType weld_profile_type, std::string name, std::string profile, double sigma) {
	LOG(INFO) << "[MakeImageProcessSteger]: constructing a new Steger processor named "
		<< name << " with type:" << common::NameOfWeldType(weld_profile_type);
	StegerProcessPtr algo;
	switch (weld_profile_type) {
	case common::WeldType::V:
		algo =  std::make_shared<StegerProcessV>(name, sigma);
		break;
	case common::WeldType::BUTT:
		algo = std::make_shared<StegerProcessButt>(name, sigma);
		break;
	case common::WeldType::CORNER:
		algo = std::make_shared<StegerProcessT>(name, sigma);
		break;
	case common::WeldType::UNKNOWN:
		algo = std::make_shared<StegerProcess>(name, sigma);
		break;
	default:
		LOG(WARNING) <<"Wrong WeldType";
	}
	if (algo) algo->setProfileModel(shape::MakeProfileModel(weld_profile_type, profile));
	return algo;
}
}