/*
 * no_toy_human_controller.cpp
 *
 *  Created on: Apr 29, 2016
 *      Author: yuxh
 *  Updated : June 15, 2016
 *      Author: GQZhao
 */

//#define Hough
//#define CannyPlusPatternFilter
#define boost_search
//
//#define pattern_track
#define boost_track

#define OpenCamera
#include "golfRecController.h"

#include <hardware/hardware_conf.h>
#include "hardware/power_hardware.h"
#include "hardware/usb_serial.h"
#include "hardware/camera_send.h"
#include "hardware/camera_hardware.h"
/*
 * no_toy_human_controller.cpp
 *
 *  Created on: Apr 29, 2016
 *      Author: yuxh
 *  Updated : June 15, 2016
 *      Author: GQZhao
 */

//#define Hough
//#define CannyPlusPatternFilter
#define boost_search
//
//#define pattern_track
#define boost_track

#define OpenCamera
#include "golfRecController.h"

#include <hardware/hardware_conf.h>
#include "hardware/power_hardware.h"
#include "hardware/usb_serial.h"
#include "hardware/camera_send.h"
#include "hardware/camera_hardware.h"
#include "hardware/camera_regs.h"
#include "modules/camera_dma_sensor.h"
#include "zoomCameraController.h"


#include "../../vision/hand/handboostingsw13.h"
//#include "../../vision/cwh_test/grab_mu_view_module.h"
#include "../../vision/ball/ball_detector.h"
#include "../../vision/golf/candidate_filter_module.h"
#include "../../vision/ball/deepnet4_ball_module.h"
//#include "../../vision/ball/boosting_ball_detector.h"
#include "../../vision/featurematch/patternmatch.h"
#include "../../vision/boost/boost_golf.h"
#include "../../vision/boost/boost_golf_8060_22.h"

#include "../../static_data/voices/iRobot/iRobot.h"
#include "../../static_data/voices/number/number.h"
#include "../../static_data/voices/golf/golf.h"

#include "../../static_data/mu_wave.h"

#include <stdlib.h>


extern speaker_wave_type speaker_wave_;

const int imwidth = 80;
const int imheight = 60;
const int deep_confirm_threshold = 30;
const int deep_search_threshold = 20;
const int boost_confirm_threshold = 300;
const int boost_confirm_threshold_level2=4500;
const int boost_search_threshold = 0;


namespace meye_products {


int setExposure(meye_drivers::CameraDMASensor* ov7725_cam,int light_value){
	//smooth light 100
//	if (light_value > 80){
//		//			  if(true){
//		ov7725_cam->AEC(0);
//		ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//	}
////	else if(light_value > 80 &&  light_value <= 100){
////		ov7725_cam->AEC(0);
////		ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 25);
////	}else if(light_value > 40 &&  light_value <= 80){
////		ov7725_cam->AEC(0);
////		ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 30);
////	}else if(light_value > 10 &&  light_value <= 40){
////		ov7725_cam->AEC(0);
////		ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 35);
////	}
//	if(light_value <40)
//		ov7725_cam->AEC(1);
//	ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
//			ov7725_cam->AEC(0);
//	ov7725_cam->AEC(0);
//
//	ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
	int exposure;
	if (light_value > 160  ){
				//			  if(true){
				ov7725_cam->AEC(0);
				ov7725_cam->AGC(0);

				ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 5);
				exposure =5;
			}

	if (light_value > 130 && light_value<=160){
			//			  if(true){
			ov7725_cam->AEC(0);
			ov7725_cam->AGC(0);

			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
			exposure =20;
		}
////		else if(light_value > 80 &&  light_value <= 140){
////			ov7725_cam->AEC(0);
////			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
////		}
		else if(light_value <= 30){
			ov7725_cam->AEC(0);
			ov7725_cam->AGC(0);

//			ov7725_cam->AGC(1);
//			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 150);
//			if(light_value<=20)
//			{
//				ov7725_cam->AGC(1);
				ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 120);
				exposure = 120;
//			}


		}
//

	return exposure;
////	 ov7725_cam->AEC(1);
//	ov7725_cam->AEC(0);
////							ov7725_cam->AGC(1);
//										ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 100);

}


void setZoom7Exposure(meye_drivers::CameraDMASensor* ov7725_cam,int light_value){
//	if (light_value > 150){
//				//			  if(true){
//				ov7725_cam->AEC(0);
//				ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//			}
//	//		else if(light_value > 80 &&  light_value <= 140){
//	//			ov7725_cam->AEC(0);
//	//			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
//	//		}
////			else if(light_value <= 90){
////				ov7725_cam->AEC(0);
////				ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
////
////				 if(light_value<70 )
////						{
////					 ov7725_cam->AEC(1);
////							ov7725_cam->AGC(1);
////						}
////			}
//	else if(light_value<120)
//	{
//		 ov7725_cam->AEC(1);
//		 if(light_value<80){
//			 ov7725_cam->AGC(1);
//		 }
//	}
//	ov7725_cam->AEC(1);
//	 ov7725_cam->AGC(1);
	if(light_value>200){
		ov7725_cam->AEC(0);
		ov7725_cam->AGC(0);

		ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 5);
}

	if (light_value > 160 && light_value<=200){
					//			  if(true){
					ov7725_cam->AEC(0);
					ov7725_cam->AGC(0);

					ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 10);

				}

	if (light_value > 130 && light_value<=150){
			//			  if(true){
		ov7725_cam->AGC(0);

			ov7725_cam->AEC(0);
			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 50);
		}
////		else if(light_value > 80 &&  light_value <= 140){
////			ov7725_cam->AEC(0);
////			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
////		}
		else if(light_value <= 80){
			ov7725_cam->AGC(0);

			ov7725_cam->AEC(1);
//			ov7725_cam->AGC(1);
//			ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 150);
//			if(light_value<=20)
//			{
//				ov7725_cam->AGC(1);
//				ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 120);

//			}


		}
//
}

int GetAverageGrayFromBuffer(int imw,int imh,int x,int y,int w,int h,const meye_os::Input& input)
{

	  uint8_t* r = input.r_;
	  uint8_t* g = input.g_;
	  uint8_t* b = input.b_;


	  int row = y;
	  int col = x;
	  int gray=0;
	  for(int k=0;k<w;k++)
		  for(int l=0;l<h;l++)
		  {
			  int value =  (30*(int)(r[ (row+k)*imwidth + col+l ]) +
					  59*(int)(g[ (row+k)*imwidth + col+l ])  +
					  11*(int)(b[ (row+k)*imwidth + col+l ])+50)/100 ;
			  gray += value;

		  }
	  gray /= (w*h);

	  return gray;

}


int NinePointLightFromBuffer(int imw,int imh,meye_os::Input& input,int light_region_x[3] ,int light_region_y[3]){
	int light_value=0;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			int x = light_region_x[j];
			int y = light_region_y[i];
			light_value += GetAverageGrayFromBuffer(	imw,imh,x,
					y,
					15,
					15,input);
		}
	light_value /= 9;
	return light_value;
}

int FourPointLightFromBuffer(meye_os::Input& input,int light_region_x[4] ,int light_region_y[4],int w,int h){
	int light_value=0;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			int x = light_region_x[j];
			int y = light_region_y[i];
			light_value += GetAverageGrayFromBuffer(	80,60,x,
					y,
					w,
					h,input);
		}
	light_value /= 4;
	return light_value;
}

int NinePointLightFromCamera(meye_drivers::CameraDMASensor* ov7725_cam,int light_region_x[9] ,int light_region_y[9]){
	int light_value=0;
	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			int x = light_region_x[j];
			int y = light_region_y[i];
			light_value += ov7725_cam->GetAverageGray(	x,
					y,
					15,
					15);

		}

	light_value /= 9;
	return light_value;
}


#define min(x,y) ((x) > (y) ? (y) : (x))
#define max(x,y) ((x) > (y) ? (x) : (y))



int abs(int a){
	return max(a,-a);

}

golfRecController::golfRecController() : golfRecController(1, 1) {}

golfRecController::golfRecController(
    uint8_t input_type_id, uint8_t input_mode_id) :
        Controller2(input_type_id, input_mode_id){

	candyfilter=NULL;
	ball_recog_module=NULL;
	pattern_tracking=NULL;
	golf_boost_search=NULL;
}

golfRecController::~golfRecController() {

  if (ball_recog_module) delete ball_recog_module;
  if (candyfilter) delete candyfilter;
  if (pattern_tracking) delete pattern_tracking;
  if (golf_boost_search) delete golf_boost_search;
}


int golfRecController::Setup() {
	if (ModeMatcher()){
		ov7725_cam = new meye_drivers::CameraDMASensor(&input_, CAMERA_FPS_20,
						CAMERA_FORMAT_RGB, CAMERA_RESOLUTION_80X60);
//		ov7725_cam->Rotate(true);


		speaker_trigger = new meye_drivers::SpeakerTrigger(
				meye_drivers::SpeakerTrigger::SPEAKER_MODE_A);

		int buffer_size = 0;

		candyfilter = new meye_vision::CandFilterModule();
		ball_recog_module = new meye_vision::DeepNet4BallModule();
		pattern_tracking = new meye_vision::PatternMatchModule();
		golf_boosting = new meye_vision::DetectorBoostingGOLF806022();
//		golf_boosting = new meye_vision::DetectorBoostingGolf();
		golf_boost_search = new meye_vision::DetectorBoostingGOLF8060();

		ball_recog_module->InitModel();
		candyfilter->InitModel();
		pattern_tracking->initModel(80,60);
		golf_boosting->InitGolfModel();
		golf_boost_search->InitGolfModel();

		int candyfilter_buff = candyfilter->CalculateBufferSize(candyfilter->subwin_w,candyfilter->subwin_h);
		int deep_buff = ball_recog_module->CalculateBufferSize(40,30);
		int track_buff = pattern_tracking->CalculateBufferSize(80,60);
		int boost_buf = golf_boosting->CalculateBufferSize(80,60);
		int boost_search_buf = golf_boost_search->CalculateBufferSize(80,60);
//		int track_buff = 0;

		buffer_size =  max(boost_search_buf ,  max (boost_buf, max(candyfilter_buff,deep_buff) ) )+ track_buff ;
		return buffer_size;
	} else {
		return -1;
	}
}

int golfRecController::MainLoop(char* buf) {
  if (buf == NULL) return 0;
  int track_buff = pattern_tracking->CalculateBufferSize(80,60);

  golf_boosting->SetBuffer(buf+track_buff);
  candyfilter->SetBuffer(buf+track_buff);
  ball_recog_module->SetBuffer(buf+track_buff);
  golf_boost_search->SetBuffer(buf+track_buff);
  pattern_tracking->SetBuffer(buf);

  speaker_wave_.speaker_wave_buffer_ =(uint8_t*) meye_static_data_iRobot::floor_board;
  speaker_wave_.speaker_wave_size_ = meye_static_data_iRobot::floor_board_length;
  speaker_trigger->Fire(state_);

  int zoom_area=0;
  int count=0;

  ov7725_cam->SetGainValue(CAMERA_VALUE_SET, 10);

  int zoom_centerx;
  int zoom_centery;
  state_.current_ball_box_.detected=false;
  bool global_search=false;
  int ball_centerx;
  int ball_centery;

  /*=================================================MainProcess=================================================*/
  state_.current_ball_box_.detected=false;


  enum state{search,track,confirm};
  state current_state = search;
  int state_record[6]={1,1,1,1};
  int state_record_num = 4;
  bool search_serve ;
  bool find_pattern = false;
  bool keep_find_pattern  = true;
  uint32_t search_loop_count = 0;
  uint32_t detected_zoom4_count = 0;
  uint32_t undetected_zoom4_count = 0;
  uint32_t search_zoom4_max_times = 7;
  uint32_t search_zoom4_stay_count_max = 4;
  uint32_t detected_zoom7_count = 0;
  uint32_t undetected_zoom7_count = 1;
  uint32_t undetected_zoom7_max_times = 5;
  bool smart_aec_flag = false;
  bool frozeExposure = false;
  int32_t smart_aec_detected_x = 65;
  int32_t smart_aec_detected_y = 45;
  int32_t smart_aec_detected_width = 15;
  int32_t smart_aec_detected_height = 15;
  int32_t smart_aec_gray = 0;
  int32_t smart_aec_gray_level_1 = 50;
  int32_t smart_aec_gray_level_2 = 100;
  int32_t smart_aec_gray_level_3 = 150;
  int32_t smart_aec_gray_level_4 = 150;
  int confirm_threshold;

  uint32_t time_start = 0;

  time_start = timeStampNow();


  int  phase=1;

//  ov7725_cam->AEC(1);
  ov7725_cam->SetGainValue(CAMERA_VALUE_SET, 0);

  int light_region_x[3] = {10,40,70};
  int light_region_y[3] = {10,30,50};

  int dtlight_count = 0;
  int light_value=0;

  int last_expo;
  int last_level = 1;
  int current_level = 1;
  int light_values[2];

  int optimal_expo=0;
  int optimal_light_value = 0;
//  for(int i=0;i<120;i++){
//	  int expo;
//	  expo = i*5;
//	  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, expo);
//	  ov7725_cam->Grab(&input_);
//
//	  light_value=NinePointLightFromBuffer(input_,light_region_x,light_region_y);
//
//	  if(abs(30-light_value)<abs(30-optimal_light_value))
//	  {
//		  optimal_expo = expo;
//		  optimal_light_value = light_value;
//	  }
//  }


//  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, optimal_expo);


//  if (light_value <= smart_aec_gray_level_1) {
//	  //					  smart_aec_flag = true;
//	  ov7725_cam->AGC(1);
//	  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 400);
//  } else {
//	  ov7725_cam->SetGainValue(CAMERA_VALUE_SET, 0);
//	  if (light_value >= smart_aec_gray_level_3) {
//		  //						  smart_aec_flag = true;
//		  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//		  last_expo = 20;
//	  } else if ((light_value >= smart_aec_gray_level_2) && \
//			  (light_value <= smart_aec_gray_level_3)) {
//		  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 100);
//		  last_expo = 100;
//	  }
//  }
//  if (light_value > 150) {
//	  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//  	  //					  smart_aec_flag = true;
////  	  ov7725_cam->AGC(1);
////  	  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 400);
//    } else {
//    	 ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 50);
//    }
//if (light_value > 100)
//	ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//else if(light_value < 5)
//	ov7725_cam->AEC(1);


ov7725_cam->AEC(0);

ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
int init_light_value[3];




int current_zoom_area;
int exposure_values[3];
int return_exposure;
  while (ModeMatcher()) {









	  int zoom7_light_value;

	  int fcount = state_.frame_count_;
	  if(fcount % 10 ==0)
		  find_pattern = false;
	  if (current_state == search) {


//		  ov7725_cam->AEC(1);
		  if(zoom_area==0 ) {
			  	  	  current_zoom_area=zoom_area;
		 			  ov7725_cam->Area(40,22,CAMERA_ZOOM_5, CAMERA_AREA_ABSOLUTE);
		 			  search_loop_count++;
		 			  phase = 1;
		 			  zoom_area = 1;

		 		  }

		  	  	  else if(zoom_area==1 ) {
			  	  	  current_zoom_area=zoom_area;

		 			  ov7725_cam->Area(40,30,CAMERA_ZOOM_5,CAMERA_AREA_ABSOLUTE);
		 			  //			  search_loop_count++;
		 			  if(phase == 1)
		 				  zoom_area = 2;
		 			  else if(phase == -1)
		 				  zoom_area = 0;
		 		  }
		  	  	  else if(zoom_area==2 ) {
			  	  	  current_zoom_area=zoom_area;

		 			  ov7725_cam->Area(40,38,CAMERA_ZOOM_5,CAMERA_AREA_ABSOLUTE);
		 			  search_loop_count++;

		 			  phase = -1;
		 			  zoom_area = 1;

		 		  }

		  int stay_count =0;
		  int detected_count = 0;

		  while ((stay_count < search_zoom4_max_times) && (detected_count == 0)) {
			  ov7725_cam->Grab(&input_);
			  state_.frame_count_++;

			  if(stay_count<=search_zoom4_stay_count_max)

			  {
			  light_value = NinePointLightFromBuffer(80,60,input_,light_region_x ,light_region_y);
			  light_values[current_zoom_area] = light_value;

//			  if(!frozeExposure)
			   exposure_values[current_zoom_area] = setExposure(ov7725_cam,light_value);

			  }

			  if(exposure_values[0]==120 &&
					  exposure_values[2]==120){
				  ov7725_cam->AGC(1);
				  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 300);

			  }
			  return_exposure = 20;
			  if(exposure_values[0]==5||exposure_values[2]==5)
				  return_exposure = 5;
//			  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 500);
//			  ov7725_cam->AEC(1);
//			  if(state_.frame_count_ >=2 && abs(light_values[0]-light_values[1])>20 )
//			  {
////				  ov7725_cam->AEC(1);
//				  ov7725_cam->SetGainValue(CAMERA_VALUE_SET, 0);
//
//				  while(count<10){
//					  light_value = NinePointLightFromCamera(ov7725_cam,light_region_x ,light_region_y);
//					  count++;
//				  }
//
//				  if (light_value <= smart_aec_gray_level_1) {
//					  //					  smart_aec_flag = true;
//					  ov7725_cam->AGC(1);
//					  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 400);
//				  } else {
//					  ov7725_cam->SetGainValue(CAMERA_VALUE_SET, 0);
//					  if (light_value >= smart_aec_gray_level_3) {
//						  //						  smart_aec_flag = true;
//						  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//					  } else if ((light_value >= smart_aec_gray_level_2) || \
//							  (light_value <= smart_aec_gray_level_3)) {
//						  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 80);
//					  }
//				  }
//
//			  }




			  //			  if(find_pattern)
			  //					  {
			  //						  pattern_tracking->Process(input_,&state_);
			  //
			  //						  smart_aec_detected_x =  pattern_tracking->pattern_centerj;
			  //						  smart_aec_detected_y =  pattern_tracking->pattern_centeri;
			  //						  smart_aec_detected_width = 5;
			  //						  smart_aec_detected_height = 5;
			  //					  }
			  if(stay_count > search_zoom4_stay_count_max) {

				  //				  smart_aec_detected_x =  state_.current_det_object_box_.center_j;
				  //				  smart_aec_detected_y =  state_.current_det_object_box_.center_j;
				  //				  smart_aec_detected_width = 5;
				  //				  smart_aec_detected_height = 5;




				  int box_size = candyfilter->box_size;
				  int outer_box_size = candyfilter->outer_box_size;
				  state_.current_ball_box_.detected=false;
				  state_.current_ball_box_.score = 0;

				  golf_boost_search->Process(input_,&state_);
				  state_.current_det_object_box_.detected = false;

				  pattern_tracking->score = 100;
				  if(find_pattern)
				 		pattern_tracking->Process(input_,&state_);
				  //


				  if (state_.current_det_object_box_.score > boost_search_threshold) {
//				  if (true){
					  //					  smart_aec_detected_x = 65;
					  //					  smart_aec_detected_y = 45;
					  //					  smart_aec_detected_width = 15;
					  //					  smart_aec_detected_height = 15;
//					  search_loop_count -- ;

					  state_.current_ball_box_.center_i= state_.current_det_object_box_.center_i;
					  state_.current_ball_box_.center_j=state_.current_det_object_box_.center_j;
					  state_.current_ball_box_.width=outer_box_size;
					  state_.current_ball_box_.height=outer_box_size;

					  detected_count++;
					  state_.current_ball_box_.detected = true;

					  meye_os::DetectedRect dtbox;

					  dtbox.center_i= state_.current_det_object_box_.center_i;
					  dtbox.center_j= state_.current_det_object_box_.center_j;
					  dtbox.width = outer_box_size;
					  dtbox.height = outer_box_size;
					  if(keep_find_pattern && (dtbox.center_i - dtbox.height/2>=0) && (dtbox.center_j - dtbox.width/2>=0))
						  pattern_tracking->pattern_init(input_,dtbox,state_);


				  }
				  else if(pattern_tracking->score<30){



					  meye_os::DetectedRect dtbox;
					  smart_aec_detected_x = 65;
					  smart_aec_detected_y = 45;
					  smart_aec_detected_width = 15;
					  smart_aec_detected_height = 15;

					  dtbox.center_i= state_.current_det_object_box_.center_i;
					  dtbox.center_j= state_.current_det_object_box_.center_j;
					  dtbox.width = outer_box_size;
					  dtbox.height = outer_box_size;
					  if(keep_find_pattern && (dtbox.center_i - dtbox.height/2>=0) && (dtbox.center_j - dtbox.width/2>=0))
						  pattern_tracking->pattern_init(input_,dtbox,state_);

					  state_.current_ball_box_.center_i= pattern_tracking->pattern_centeri;
					  state_.current_ball_box_.center_j=pattern_tracking->pattern_centerj;
					  state_.current_ball_box_.width=outer_box_size;
					  state_.current_ball_box_.height=outer_box_size;



					  ball_recog_module->Process(input_,&state_);
					  if (state_.current_ball_box_.score >= deep_search_threshold) {
						  detected_count++;
						  state_.current_ball_box_.detected = true;
					  }

					  //					  }
				  }
				  //		else{
				  //					  uint8_t* r = input_.r_;
				  //					  uint8_t* g = input_.g_;
				  //					  uint8_t* b = input_.b_;
				  //
				  //					  int row = 0;
				  //					  int col = 0;
				  //					  int max_hypo_x=0;
				  //					  int max_hypo_y=0;
				  //					  int max_hypo_value=0;
				  //					  while(row<6 ){
				  //						  col = 0;
				  //						  while(col<imwidth-6 )
				  //						  {
				  //							  int hypo_point_value=0;
				  //							  for(int k=0;k<6;k++)
				  //								  for(int l=0;l<6;l++)
				  //								  {
				  //									  double value =  3*r[ (row+k)*imwidth + col+l ]/10 +
				  //											  59*g[ (row+k)*imwidth + col+l ]/100  +
				  //											  11*b[ (row+k)*imwidth + col+l ]/100 ;
				  //									  hypo_point_value += value;
				  //
				  //								  }
				  //							  hypo_point_value /= 36;
				  //							  if(hypo_point_value >= max_hypo_value){
				  //								  max_hypo_value = hypo_point_value;
				  //								  max_hypo_x = col;
				  //								  max_hypo_y = row;
				  //							  }
				  //							  col += 1;
				  //						  }
				  //						  row += 1;
				  //					  }
				  //					  if(max_hypo_value>180){
				  //						  detected_count++;
				  //
				  //						  state_.current_ball_box_.center_i= max_hypo_y+2;
				  //						  state_.current_ball_box_.center_j= max_hypo_x+2;
				  //					  }
				  //
				  //				  }


				  //				  if ((smart_aec_flag == true) //&& \
				  //						  (state_.current_det_object_box_.score <= boost_search_threshold || state_.current_det_object_box_.score < deep_search_threshold)
				  //						  )
				  //				  {
//				  					  ov7725_cam->SmartAEC(smart_aec_detected_x,
				  //							  smart_aec_detected_y,
				  //							  smart_aec_detected_width,
				  //							  smart_aec_detected_height,
				  //							  smart_aec_gray);
				  //				  }

			  }
			  state_.PrintState();
			  FrameOutput(&input_, &state_, false);
			  stay_count++;
		  }

		  if (detected_count > 0) {

//			  ov7725_cam->AEC(1);
//			  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 100);


//			  smart_aec_gray = ov7725_cam->GetAverageGray(smart_aec_detected_x,
//					  smart_aec_detected_y,
//					  smart_aec_detected_width,
//					  smart_aec_detected_height);
			  current_state = confirm;
			  ball_centerx =   state_.current_ball_box_.center_j;
			  ball_centery =   state_.current_ball_box_.center_i;
			  int zoom4_light_region_x[2] = {ball_centerx -15,ball_centerx + 15};
			  int zoom4_light_region_y[2] = {ball_centery -15,ball_centery + 15};

//			  if(zoom4_light_region_x[0]>=0 && \
//					  zoom4_light_region_x[1]>=0 && \
//					  zoom4_light_region_y[0]>=0 && \
//					  zoom4_light_region_y[1]>=0 && \
//					  zoom4_light_region_x[0]<=79 && \
//					  zoom4_light_region_x[1]<=79 && \
//					  zoom4_light_region_y[0]<=79 && \
//					  zoom4_light_region_y[1]<=79)
//			  {
//				  int zoom4_light_value = FourPointLightFromBuffer(input_,zoom4_light_region_x ,zoom4_light_region_y,8,8);
//				  setZoom7Exposure(ov7725_cam,zoom4_light_value);
//			  }

			  ov7725_cam->Area(ball_centerx, ball_centery,CAMERA_ZOOM_7_VGA, CAMERA_AREA_RELATIVE);
			  LED_RGB(1,0,1);
		  }
//		  if (search_loop_count >= 3 ) {
//					  speaker_wave_.speaker_wave_buffer_ =	(uint8_t*) meye_static_data_golf::serve;
//					  speaker_wave_.speaker_wave_size_ 	=  meye_static_data_golf::serve_length;
//					  speaker_trigger->Fire(state_);
//					  search_loop_count = 0;
//				  }

//		  count += 1;
//		  zoom_area = count % 2;
	  }

	  if (current_state == confirm) {
		  bool zoom_smart_aec_flag;
//		  ov7725_cam->AEC(0);

//
		  zoom_smart_aec_flag = false;


//		  int zoom7_light_value = FourPointLightFromBuffer(input_,zoom7_light_region_x ,zoom7_light_region_y);
//		  setZoom7Exposure(ov7725_cam,zoom7_light_value);

//		  int zoom7_light_region_x[2] = {0,65};
//		  int zoom7_light_region_y[2] = {0,45};
//		  zoom7_light_value = FourPointLightFromBuffer(input_,zoom7_light_region_x ,zoom7_light_region_y,15,15);
//
//		  setExposure(ov7725_cam,zoom7_light_value);


		  confirm_threshold = 1;
//		  search_loop_count = 0;
		  state_.current_ball_box_.detected=false;
		  state_.current_ball_box_.score =0;

		  //      ov7725_cam->SyncedGrab(&input_);
		  ov7725_cam->Grab(&input_);
		  state_.frame_count_++;
//		  if(undetected_zoom7_count > 0)
		  golf_boosting->Process(input_,&state_);

		  int dtcenterx = state_.current_det_object_box_.center_j;
		  int dtcentery = state_.current_det_object_box_.center_i;
		  int dtw =  state_.current_det_object_box_.width;
		  int dth =  state_.current_det_object_box_.height;

		  if(			state_.current_det_object_box_.score>boost_confirm_threshold
				  && 	state_.current_det_object_box_.score<=boost_confirm_threshold_level2) {

			  state_.current_ball_box_.center_i=  dtcentery ;
			  state_.current_ball_box_.center_j=  dtcenterx ;
			  if(current_zoom_area==0){
			  state_.current_ball_box_.width= dtw;
			  state_.current_ball_box_.height= dth;}
			  if(current_zoom_area == 1){
			  state_.current_ball_box_.width= dtw+3;
			  state_.current_ball_box_.height= dth+3;}
			  if(current_zoom_area == 2){
			  state_.current_ball_box_.width= dtw+6;
			  state_.current_ball_box_.height= dth+6;}
			  ball_recog_module->Process(input_,&state_);
			  confirm_threshold = deep_confirm_threshold;

		  }else if( state_.current_det_object_box_.score>boost_confirm_threshold_level2){

			  state_.current_ball_box_.center_i=  dtcentery ;
			  state_.current_ball_box_.center_j=  dtcenterx ;

			  state_.current_ball_box_.score = state_.current_det_object_box_.score;
			  confirm_threshold = boost_confirm_threshold;
		  }

		  state_.current_det_object_box_.detected = false;


		  if (state_.current_ball_box_.score >= confirm_threshold) {
			  frozeExposure = true;
			  ov7725_cam->AEC(0);
			  keep_find_pattern=false;
			  find_pattern = true;
			  zoom_smart_aec_flag = false;
			  state_.current_ball_box_.detected=true;
			  speaker_wave_.speaker_wave_buffer_ =	(uint8_t*) meye_static_data_golf::ball;
			  speaker_wave_.speaker_wave_size_ 	=  meye_static_data_golf::ball_length;
			  speaker_trigger->Fire(state_);

			  detected_zoom7_count++;
			  undetected_zoom7_count = 0;
			  search_loop_count =0;
//			  ov7725_cam->Area(state_.current_ball_box_.center_j,
//									 state_.current_ball_box_.center_i,
//									 CAMERA_ZOOM_7_VGA,
//									 CAMERA_AREA_RELATIVE);
		  }
//		  else {
//		        undetected_zoom7_count++;
//		        if (undetected_zoom7_count > undetected_zoom7_max_times) {
//		          detected_zoom7_count = 0;
//		          undetected_zoom7_count = 0;
////		          if (smart_aec_flag == true) {
////		            ov7725_cam->AEC(0);
////		          }
////		          zoom_area = !zoom_area;
//		          current_state = search; //
//		        }
//		      }
//
//		      if (
//		          ((state_.current_det_object_box_.score < boost_search_threshold) || \
//		          (state_.current_ball_box_.score < deep_search_threshold))) {
//		    	  int32_t zoom7_light_region_x[2] = {0,65};
//		    	  int32_t zoom7_light_region_y[2] = {0,45};
//		          ov7725_cam->FourPointSmartAEC(zoom7_light_region_x,
//		        		  zoom7_light_region_y,
//		                               10,
//		                               10,
//		                               90);
//		      }


//
		  else {


			  keep_find_pattern=true;
//			  find_pattern = false;
			  zoom_smart_aec_flag = true;
			  undetected_zoom7_count++;
			  if (undetected_zoom7_count > undetected_zoom7_max_times) {
				  detected_zoom7_count = 0;
				  undetected_zoom7_count = 1;
//				  if (zoom_smart_aec_flag == true) {
//					  ov7725_cam->AEC(0);
//				  }


//				  zoom_area += 1;
//				  if(zoom_area == 0)
//					  zoom_area = 1 ;
//				  else if(zoom_area == 1)
//					  zoom_area = 0 ;
//				  else
//					  zoom_area = 1 ;
//				  if(zoom_area == 0)
//					  zoom_area = 1;
//				  else if(zoom_area == 2)
//					  zoom_area =1;
//				  else if(zoom_area == 1){
//					  if(phase == 1)
//						  zoom_area = 2;
//					  if(phase == -1)
//						  zoom_area = 0;
//				  }
				  current_state = search;
				  ov7725_cam->AEC(0);
				  ov7725_cam->AGC(0);

				  ov7725_cam->SetExposureValue( CAMERA_VALUE_SET, return_exposure );

				  return_exposure = exposure_values[zoom_area];
//				  search_loop_count++;
//				  if(zoom_area==1)
//					  search_loop_count++;
			  }
		  }

		  if(detected_zoom7_count==0)
		  		  {

		  			  int zoom7_light_region_x[2] = {0,65};
		  			  int zoom7_light_region_y[2] = {0,45};
		  			  zoom7_light_value = FourPointLightFromBuffer(input_,zoom7_light_region_x ,zoom7_light_region_y,10,10);

		  			  setZoom7Exposure(ov7725_cam,zoom7_light_value);
		  		  }

//		  if ((zoom_smart_aec_flag == true) && \
//				  state_.current_ball_box_.score < confirm_threshold) {
//			  int zoom7_light_region_x[2] = {0,65};
//			  int zoom7_light_region_y[2] = {0,45};
//
//			  int zoom7_light_value = NinePointLightFromBuffer(input_,zoom7_light_region_x ,zoom7_light_region_y);
//			  if (zoom7_light_value > 130){
//				  ov7725_cam->AEC(0);
//				  ov7725_cam->SetExposureValue(CAMERA_VALUE_SET, 20);
//			  }
//			  if(zoom7_light_value <70)
//					ov7725_cam->AEC(1);
//			  ov7725_cam->SmartAEC(smart_aec_detected_x,
//					  smart_aec_detected_y,
//					  smart_aec_detected_width,
//					  smart_aec_detected_height,
//					  smart_aec_gray);
//		  }
		  state_.PrintState();
		  FrameOutput(&input_, &state_, false);
		  //			  FrameSendToSerial(&input_, &state_);
	  }

	  if (search_loop_count >= 2 && current_state == search ) {
			  speaker_wave_.speaker_wave_buffer_ =	(uint8_t*) meye_static_data_golf::serve;
			  speaker_wave_.speaker_wave_size_ 	=  meye_static_data_golf::serve_length;
			  speaker_trigger->Fire(state_);
			  search_loop_count = 0;
		  }
//	  if(current_state == confirm)
//	  {
//		  state_.current_ball_box_.detected=false;
//		  state_.current_ball_box_.score =0;
//
//		  golf_boosting->Process(input_,&state_);
//		  int dtcenterx = state_.current_det_object_box_.center_j;
//		  int dtcentery = state_.current_det_object_box_.center_i;
//		  int dtw =  state_.current_det_object_box_.width;
//		  int dth =  state_.current_det_object_box_.height;
//
//
//		  if(state_.current_det_object_box_.score>boost_confirm_threshold)
//		  {
//			  state_.current_ball_box_.center_i=  dtcentery ;
//			  state_.current_ball_box_.center_j=  dtcenterx ;
//			  state_.current_ball_box_.width= dtw+6;
//			  state_.current_ball_box_.height= dth+6;
//			  ball_recog_module->Process(input_,&state_);
//		  }
//
//		  state_.current_det_object_box_.detected = false;
//
//		  if(state_.current_ball_box_.score >=deep_confirm_threshold)
//		  {
//			  state_.current_ball_box_.detected=true;
//			  speaker_wave_.speaker_wave_buffer_ =	(uint8_t*) meye_static_data_golf::ball;
//			  speaker_wave_.speaker_wave_size_ 	=  meye_static_data_golf::ball_length;
//			  speaker_trigger->Fire(state_);
//
//			  current_state = confirm;
//			  fcount+=1;
//			  state_record[fcount % state_record_num ] = 1;
//
//			    ov7725_cam->SmartAEC(	state_.current_ball_box_.center_j,
//										state_.current_ball_box_.center_i,
//										18,
//										18,
//				gray);
//
//		  }
//
//
//		  if(state_.current_ball_box_.score <deep_confirm_threshold)
//		  {
//			  fcount+=1;
//			  state_record[fcount % state_record_num ] = -1;
//			  current_state = search;
//			  state_.PrintState();
//			  FrameOutput(&input_, &state_, false);
////			  FrameSendToSerial(&input_, &state_);
//			  continue;
//		  }
//
//	  }





//	  if(current_state == search){
//
//		  int stay_count =0;
//		  int detected_count = 0;
//		  ov7725_cam->SyncedGrab(&input_);
//		  while(stay_count<6){
//
//			  if(stay_count>=2)
//			  {
//			  state_.current_ball_box_.score = 0;
//			  int box_size = candyfilter->box_size;
//			  int outer_box_size = candyfilter->outer_box_size;
//			  meye_os::DetectedRect track_box;
//			  state_.current_ball_box_.detected=false;
//
//			  state_.current_ball_box_.score = 0;
//			  golf_boost_search->Process(input_,&state_);
//			  state_.current_det_object_box_.detected = false;
//
//			  if(state_.current_det_object_box_.score>boost_search_threshold)
//			  {
//				  state_.current_ball_box_.center_i= state_.current_det_object_box_.center_i;
//				  state_.current_ball_box_.center_j=state_.current_det_object_box_.center_j;
//				  state_.current_ball_box_.width=outer_box_size;
//				  state_.current_ball_box_.height=outer_box_size;
//
//
//				  ball_recog_module->Process(input_,&state_);
//
//
//				  if( state_.current_ball_box_.score>deep_search_threshold)
//				  {
//					  detected_count+=1;
//					  state_.current_ball_box_.detected = true;
//
//				  }
//			  }
//		  }
//			  stay_count+=1;
//
//		  }
//
//
//		  if(detected_count==0)
//		  {
//			  current_state = search;
//			  state_record[fcount % state_record_num ] = 0;
//
//		  }
//
//		  if(detected_count>0)
//		  {
//
//
//			  ov7725_cam->AEC(0);
//			  gray = ov7725_cam->GetAverageGray(	  state_.current_ball_box_.center_j,
//													  state_.current_ball_box_.center_i,
//													  8,
//													  8);
//			  state_record[fcount % state_record_num ] = 1;
//			  current_state = confirm;
//			  ball_centerx =   state_.current_ball_box_.center_j;
//			  ball_centery =   state_.current_ball_box_.center_i;
//			  ov7725_cam->Area(   ball_centerx, ball_centery,CAMERA_ZOOM_7_VGA, CAMERA_AREA_RELATIVE);
//			  LED_RGB(1,0,1);
//
//		  }
//		  fcount+=1;
//
//	  }



//	  if(current_state == search)
//	  {
//		  count += 1;
//		  zoom_area = count % 2;
//	  }



//	  search_serve  = true;
//	  for (int i=0; i<state_record_num ;i ++)
//	  {
//		  if(state_record[i] == 1 || state_record[i] == -1)
//		  {
//			  search_serve = false;
//		  }
//	  }


//	  if(search_serve)
//	  {
//
//		  speaker_wave_.speaker_wave_buffer_ =	(uint8_t*) meye_static_data_golf::serve;
//		  speaker_wave_.speaker_wave_size_ 	=  meye_static_data_golf::serve_length;
//		  speaker_trigger->Fire(state_);
//		  current_state = search;
//
//	  }







//#ifdef	OpenCamera
//
//	  state_.PrintState();
//
////	  FrameSendToSerial(&input_, &state_);
//
//	  FrameOutput(&input_, &state_, false);
//#endif
//	  state_.frame_count_++;

  } // while()
  return 1;
}


void golfRecController::RunIfModeMatch(
    uint8_t input_type_id, uint8_t input_mode_id){
	golfRecController human_controller(input_type_id, input_mode_id);
  int buffer_size = human_controller.Setup();
  if (buffer_size >= 0) {
    char buf[buffer_size];
    human_controller.MainLoop(buf);
  }
}

}  // namespace meye_products.

