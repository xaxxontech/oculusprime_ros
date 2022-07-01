#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>


ros::Publisher pub;

double camFOVy;
const float frame_z_ = 0.29; // z height of camera off floor     TODO: config/arg set
const double floorplane_obstacle_height_ = 0.04; // 0.075  TODO: config/arg set
const double floorplane_cliff_depth_ = 0.04; // 0.075   TODO: config/arg set

bool calibrationcomplete = false;
int calibrationscancount = 0;
float avghrzangleoffst = 0.0;
float horiz_angle_offset_ = 0.0;
bool framecalibrated;
const float HRZCALRANGE = 0.12217; // +/- vert radians (7 deg)
const float HRZCALINC = 0.00576; // 0.33 deg
const int CALIBFRAMESMAX = 5;
int framecount = 0;
const int DROPINITFRAMES = 30;
const float CALIBCONST = -0.02; // horiz_angle_offset_ distortion comp
const int SKIPY = 2;
const int SKIPX = 1;
const float RANGE_MIN = 0.4;   // TODO: config/arg set
const float RANGE_MAX_HORIZONTAL = 6.0;    // TODO: config/arg set
const float RANGE_MAX_NON_HORIZONTAL = 3.0;    // TODO: config/arg set


bool use_point(const float new_value, const float old_value, 
		const float range_min, const float range_max) {
			  
  // Check for NaNs and Infs, a real number within our limits is more desirable than these.
  bool new_finite = std::isfinite(new_value);
  bool old_finite = std::isfinite(old_value);
  
  // Infs are preferable over NaNs (more information)
  if(!new_finite && !old_finite){ // Both are not NaN or Inf.
    if(!isnan(new_value)){ // new is not NaN, so use it's +-Inf value.
      return true;
    }
    return false; // Do not replace old_value
  }
  
  // If not in range, don't bother
  bool range_check = range_min <= new_value && new_value <= range_max;
  if(!range_check){
    return false;
  }
  
  if(!old_finite){ // New value is in range and finite, use it.
    return true;
  }
  
  // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
  bool shorter_check = new_value < old_value;
  return shorter_check;
}


double magnitude_of_ray(const cv::Point3d& ray) {
  return sqrt(pow(ray.x, 2.0) + pow(ray.y, 2.0) + pow(ray.z, 2.0));
}


double angle_between_rays(const cv::Point3d& ray1, const cv::Point3d& ray2) {
  double dot_product = ray1.x*ray2.x + ray1.y*ray2.y + ray1.z*ray2.z;
  double magnitude1 = magnitude_of_ray(ray1);
  double magnitude2 = magnitude_of_ray(ray2);;
  return acos(dot_product / (magnitude1 * magnitude2));
}


/* prep the scan msg */   
sensor_msgs::LaserScanPtr get_scan_msg(const sensor_msgs::ImageConstPtr& depth_msg,
		const image_geometry::PinholeCameraModel cam_model_) {

	
	//   Calculate angle_min and angle_max by measuring angles between the left ray, right ray, and optical center ray
	cv::Point2d raw_pixel_left(0, cam_model_.cy());
	cv::Point2d rect_pixel_left = cam_model_.rectifyPoint(raw_pixel_left);
	cv::Point3d left_ray = cam_model_.projectPixelTo3dRay(rect_pixel_left);

	cv::Point2d raw_pixel_right(depth_msg->width-1, cam_model_.cy());
	cv::Point2d rect_pixel_right = cam_model_.rectifyPoint(raw_pixel_right);
	cv::Point3d right_ray = cam_model_.projectPixelTo3dRay(rect_pixel_right);

	cv::Point2d raw_pixel_center(cam_model_.cx(), cam_model_.cy());
	cv::Point2d rect_pixel_center = cam_model_.rectifyPoint(raw_pixel_center);
	cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(rect_pixel_center);

	double angle_max = angle_between_rays(left_ray, center_ray);
	double angle_min = -angle_between_rays(center_ray, right_ray); // Negative because the laserscan message expects an opposite rotation of that from the depth image

	// Calculate vertical field of view angle
	cv::Point2d raw_pixel_top(0, cam_model_.cx());
	cv::Point2d rect_pixel_top = cam_model_.rectifyPoint(raw_pixel_top);
	cv::Point3d top_ray = cam_model_.projectPixelTo3dRay(rect_pixel_top);

	cv::Point2d raw_pixel_bottom(depth_msg->height-1, cam_model_.cx());
	cv::Point2d rect_pixel_bottom = cam_model_.rectifyPoint(raw_pixel_bottom);
	cv::Point3d bottom_ray = cam_model_.projectPixelTo3dRay(rect_pixel_bottom);

	camFOVy = angle_between_rays(top_ray, bottom_ray);

	// populate laserscan message header
	sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan());
	scan_msg->header = depth_msg->header;

	scan_msg->header.frame_id = "camera_depth_frame"; // TODO: config/arg set

	scan_msg->angle_min = angle_min;
	scan_msg->angle_max = angle_max;
	scan_msg->angle_increment = (scan_msg->angle_max - scan_msg->angle_min) / (depth_msg->width - 1);
	scan_msg->time_increment = 0.0;
	scan_msg->scan_time = 0.0;
	scan_msg->range_min = RANGE_MIN;
	scan_msg->range_max = RANGE_MAX_HORIZONTAL;
	
	// Calculate and fill the ranges
	uint32_t ranges_size = depth_msg->width;
	scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::quiet_NaN());

	return scan_msg;
}


/* extract data from depth image */
void imageCb(const sensor_msgs::ImageConstPtr& depth_msg, 
   const sensor_msgs::CameraInfoConstPtr& info_msg) {
	
	// skip initial frames to ensure stable image, good calibration    
	if (framecount < DROPINITFRAMES) {
		framecount++;
		return;
	}
	
	if (depth_msg->width <1) return;  // TODO: testing

	// setup constants
	image_geometry::PinholeCameraModel cam_model_;
	cam_model_.fromCameraInfo(info_msg);
	
	sensor_msgs::LaserScanPtr scan_msg = get_scan_msg(depth_msg, cam_model_);
	const int MAXINDEX = depth_msg->width -1;
	
	const float center_x = cam_model_.cx();
	const float center_y = cam_model_.cy(); // pixels

	const double unit_scaling = 0.001f;
	const float constant_x = unit_scaling / cam_model_.fx();
	const float constant_y = unit_scaling / cam_model_.fy();

	const int row_step = depth_msg->step / sizeof(uint16_t);
	const int vhorizmax = (int) (depth_msg->height/2) + 5;
	const int vhorizmin = (int) (depth_msg->height/2) - 5;

	float hrzangleoffst;
	double fpobstacleheight;
	double fpcliffdepth;

	// use tighter FP tolerance for calibration 
	if (calibrationcomplete) {
		hrzangleoffst = horiz_angle_offset_; 
		fpobstacleheight = floorplane_obstacle_height_;
		fpcliffdepth = floorplane_cliff_depth_;
		framecalibrated = true;
	} else {
		framecalibrated = false;
		hrzangleoffst = -HRZCALRANGE-CALIBCONST;
		fpobstacleheight = 0.01;
		fpcliffdepth = 0.01;
	}
	
	int frameFPcount = 0;
	int maxframeFPcount = 0;
	float winnerhrzangleoffst = 0;

	// iterate through image pixels, multiple on initial frames for calibration, once only after
	do {
		const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
		
		for (int v = 0; v < depth_msg->height; v++, depth_row += row_step) { 

			if ( !(v % SKIPY) ) continue; // && !framecalibrated // floor plane skip % of vert pixels reduce cpu

			for (int u = 0; u < (int) depth_msg->width; u+=SKIPX) {  

				uint16_t depth = depth_row[u];

				double r = depth; // Assign to pass through NaNs and Infs
				double th = -atan2((double)(u - center_x) * constant_x, unit_scaling); // Atan2(x, z), but depth divides out
				int index = (th - scan_msg->angle_min) / scan_msg->angle_increment;
				if (index > MAXINDEX) index = MAXINDEX;
				if (index < 0) index = 0;

				if (depth != 0) {
					// Calculate in XYZ
					double x = (u - center_x) * depth * constant_x;
					double z = depth * unit_scaling;
					
					// ignore floor plane points
					double yAngle = (v - center_y) * camFOVy/depth_msg->height;
					yAngle += hrzangleoffst;
					float fpMin = (frame_z_ - fpobstacleheight) / sin(yAngle);
					float fpMax = (frame_z_ + fpcliffdepth) / sin(yAngle);
					if (z>fpMin && z<fpMax) {
						if (z <= 1.0)  frameFPcount ++; // (for calibration use only)
						continue;  // is within floor plane, so skip use_point
					}
					
					// Calculate actual distance
					r = sqrt(pow(x, 2.0) + pow(z, 2.0));
				}
				
				// if y is within horiz ctr range (like lidar), use far range max
				// if not, use closer range
				float rangemax = RANGE_MAX_NON_HORIZONTAL;
				if (v <= vhorizmax && v >= vhorizmin) rangemax = RANGE_MAX_HORIZONTAL;
				
			
				// Determine if this point should be used.
				if(use_point(r, scan_msg->ranges[index], RANGE_MIN, rangemax)){
					scan_msg->ranges[index] = r;
				}
				
			} // end of u loop

		} // end of v loop
		
			
		if (frameFPcount > maxframeFPcount && !framecalibrated) {
			maxframeFPcount = frameFPcount;
			winnerhrzangleoffst = hrzangleoffst;
		}
		
		if (!framecalibrated) {
			// std::cout << "hrzangleoffst: " << hrzangleoffst << ", frameFPcount: " << frameFPcount << "\n";

			
			hrzangleoffst += HRZCALINC;
			
			if (hrzangleoffst > HRZCALRANGE-CALIBCONST) { // frame calibrated
				framecalibrated = true;
				avghrzangleoffst += winnerhrzangleoffst;
				calibrationscancount ++;
				
				std::cout << "frame calibrated: " << winnerhrzangleoffst + CALIBCONST << "\n";
				std::cout << "maxframeFPcount: " << maxframeFPcount << "\n";
				
				if (calibrationscancount >= CALIBFRAMESMAX) {
					calibrationcomplete = true;
					horiz_angle_offset_ = avghrzangleoffst/CALIBFRAMESMAX + CALIBCONST;
					
					std::cout << "calibration complete " << horiz_angle_offset_ << "\n";
				}
				
			}
		}
		
		frameFPcount = 0;
		
	} while (!framecalibrated);
	
	pub.publish(scan_msg);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "depthcamtoscan");
	ros::NodeHandle nh;

	pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);  // TODO: config/arg set

	image_transport::ImageTransport it(nh);  
	image_transport::CameraSubscriber sub = it.subscribeCamera("/camera/depth/image_rect_raw", 1, imageCb);   // TODO: config/arg set

	ros::spin();
}

