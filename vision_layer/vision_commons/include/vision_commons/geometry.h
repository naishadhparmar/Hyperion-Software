#ifndef blue_filter_H
#define blue_filter_H

#include "opencv2/core/core.hpp"

namespace vision_commons {
	class Geometry {
		public:
			static double distance(
				cv::Point p1,
				cv::Point p2
			);
			static double angleWrtY(
				cv::Point p1,
				cv::Point p2
			);
	};
}

#endif
