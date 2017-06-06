/*
 * no_toy_human_controller.cpp
 *
 *  Created on: Apr 29, 2016
 *      Author: yuxh
 *  Updated : June 15, 2016
 *      Author: GQZhao
 */

#ifndef SRC_PRODUCTS_NO_TOY_FLOOR_CONTROLLER_H_
#define SRC_PRODUCTS_NO_TOY_FLOOR_CONTROLLER_H_


#include "meye_os/os_controller2.h"
#include "meye_os/os_module.h"
#include "modules/camera_dma_sensor.h"
#include "modules/led_trigger.h"
#include "modules/servo_trigger.h"
#include "modules/speaker_trigger.h"

#include "../../vision/ball/ball_detector.h"
//#include "../vision/cwh_test/grab_mu_view_module.h"
#include "../../vision/ball/deepnet4_ball_module.h"
#include "../../vision/ballboosting/ballboostingsw.h"
#include "../../vision/featurematch/framediffmodule.h"
#include "../../vision/golf/candidate_filter_module.h"
#include "../../vision/featurematch/patternmatch.h"
//#include "../../vision/boost/boost_golf.h"
#include "../../vision/boost/boost_golf_8060.h"
#include "../../vision/boost/boost_golf_8060_22.h"


//include <static_data/mu_color.h>

#define  _REG_MHZ_NORMAL_VAL_                   REG_18MHZ
#define  _REG_MHZ_LOWPOWER_VAL_                 REG_18MHZ

namespace meye_products {

class golfRecController : public meye_os::Controller2 {

 public:
	golfRecController();

	golfRecController(uint8_t input_type_id, uint8_t input_mode_id);
	  int exposure_level = 0;
  virtual ~golfRecController();

  virtual int Setup();
  virtual int MainLoop(char* buf);
  static void RunIfModeMatch(uint8_t input_type_id, uint8_t input_mode_id);
//  const uint8_t pattern[11*11];
 protected:
//  meye_os::Module* hand_boost_module;
  meye_drivers::CameraDMASensor* ov7725_cam;

  meye_vision::BallDetector* hough_circle_detector;
  meye_vision::BallDetectorSW1010* ballboosting;
  meye_vision::FrameDiffModule* movingDetect;
  meye_vision::CandFilterModule* candyfilter;
  meye_vision::DeepNet4BallModule* ball_recog_module;
  meye_vision::PatternMatchModule* pattern_tracking;
//  meye_vision::DetectorBoostingGolf * golf_boosting;
  meye_vision::DetectorBoostingGOLF806022* golf_boosting;
  meye_vision::DetectorBoostingGOLF8060* golf_boost_search;





};

}  // namespace meye_products


#endif /* SRC_PRODUCTS_NO_TOY_HAND_CONTROLLER_H_ */
