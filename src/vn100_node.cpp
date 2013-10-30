
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>     /* exit, EXIT_FAILURE */

#include "vectornav.h"

#include <ros/ros.h>
#include <tf/tf.h>

// Message Types
#include <vectornav/gps.h>
#include <vectornav/ins.h>
#include <vectornav/sensors.h>

// Params
std::string imu_frame_id, gps_frame_id;

// Publishers
ros::Publisher pub_sensors;

Vn100 vn100;

void asyncDataListener(Vn100* sender, Vn100CompositeData* data)
{
  //TODO: Publish messages perhaps? ;)

  ros::Time timestamp =  ros::Time::now();
  static int seq = 0;
  seq++;
  // Debug message
  ROS_INFO("\nASYNC Data:\n"
	  "  YPR.Yaw:                %+#7.2f\n"
	  "  YPR.Pitch:              %+#7.2f\n"
	  "  YPR.Roll:               %+#7.2f\n" ,
	  data->ypr.yaw,
    data->ypr.pitch,
    data->ypr.roll);
    
    ROS_INFO(
	  "\n  quaternion.X:           %+#7.2f\n"
	  "  quaternion.Y:           %+#7.2f\n"
	  "  quaternion.Z:           %+#7.2f\n"
	  "  quaternion.W:           %+#7.2f\n",
	  data->quaternion.x,
	  data->quaternion.y,
	  data->quaternion.z,
	  data->quaternion.w);
	  
    ROS_INFO(
	  "\n                          {Value, Voltage}\n"
	  "  magnetic X:             %+#7.2f, %+#7.2f\n"
	  "  magnetic Y:             %+#7.2f, %+#7.2f\n"
	  "  magnetic Z:             %+#7.2f, %+#7.2f\n",
	  data->magnetic.c0, data->magneticVoltage.c0, 
	  data->magnetic.c1, data->magneticVoltage.c1, 
	  data->magnetic.c2, data->magneticVoltage.c2);

    ROS_INFO(
	  "\n  acceleration X:         %+#7.2f, %+#7.2f\n"
	  "  acceleration Y:         %+#7.2f, %+#7.2f\n"
	  "  acceleration Z:         %+#7.2f, %+#7.2f\n",
	  data->acceleration.c0, data->accelerationVoltage.c0, 
	  data->acceleration.c1, data->accelerationVoltage.c1, 
	  data->acceleration.c2, data->accelerationVoltage.c2);

    ROS_INFO(
	  "\n                          {Value, Voltage, Bias, BiasVariance}\n"
	  "  angularRate X:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Y:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "  angularRate Z:          %+#7.2f, %+#7.2f, %+#7.2f, %+#7.2f\n",
	  data->angularRate.c0,     data->angularRateVoltage.c0, 
	  data->angularRateBias.c0, data->angularRateBiasVariance.c0, 
	  data->angularRate.c1,     data->angularRateVoltage.c1, 
	  data->angularRateBias.c1, data->angularRateBiasVariance.c1, 
	  data->angularRate.c2,     data->angularRateVoltage.c2,
	  data->angularRateBias.c2, data->angularRateBiasVariance.c2);

    ROS_INFO(
	  "\n  Attitude Variance X:    %+#7.2f\n"
	  "  Attitude Variance Y:    %+#7.2f\n"
	  "  Attitude Variance Z:    %+#7.2f\n",
	  data->attitudeVariance.c0, 
	  data->attitudeVariance.c1, 
	  data->attitudeVariance.c2);

    ROS_INFO(
	  "\n  Direction Cosine Matrix:\n"
	  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "    %+#7.2f, %+#7.2f, %+#7.2f\n"
	  "    %+#7.2f, %+#7.2f, %+#7.2f\n",
	  data->dcm.c00, data->dcm.c01, data->dcm.c02,
	  data->dcm.c10, data->dcm.c11, data->dcm.c12,
	  data->dcm.c20, data->dcm.c21, data->dcm.c22);

    ROS_INFO(
	  "\n  Temperature:            %+#7.2f\n"
	  "  Temperature Voltage:    %+#7.2f\n",
	  data->temperature,
	  data->temperatureVoltage);

}

void poll_device()
{
    // Only bother if we have subscribers
    if (pub_sensors.getNumSubscribers()     <= 0 )
    {
      return;
    }
    
    static int seq = 0;
    seq++;
    ros::Time timestamp =  ros::Time::now(); 
    
    // IMU Data
    //if (pub_sensors.getNumSubscribers() > 0)
    //{
      VnVector3 magnetic, acceleration, angularRate;
      float temperature, pressure;
      
      vn100_getCalibratedSensorMeasurements(  &vn100,
                                              &magnetic,
                                              &acceleration,
                                              &angularRate,
                                              &temperature,
                                              &pressure );

      vectornav::sensors msg_sensors;
      msg_sensors.header.seq      = seq;
      msg_sensors.header.stamp    = timestamp;
      msg_sensors.header.frame_id = imu_frame_id;

      msg_sensors.Mag.x = magnetic.c0;
      msg_sensors.Mag.y = magnetic.c1;
      msg_sensors.Mag.z = magnetic.c2;

      msg_sensors.Accel.x = acceleration.c0;
      msg_sensors.Accel.y = acceleration.c1;
      msg_sensors.Accel.z = acceleration.c2;

      msg_sensors.Gyro.x = angularRate.c0;
      msg_sensors.Gyro.y = angularRate.c1;
      msg_sensors.Gyro.z = angularRate.c2;
      
      msg_sensors.Temp     = temperature;
      msg_sensors.Pressure = pressure;
      
      pub_sensors.publish(msg_sensors);
      
    //}
}
void poll_timerCB(const ros::TimerEvent&)
{
  poll_device();
}

void vnerr_msg(VN_ERROR_CODE vn_error, char* msg)
{
  switch(vn_error)
  {
    case VNERR_NO_ERROR:
      strcpy(msg, "No Error");
      break;
    case VNERR_UNKNOWN_ERROR:
      strcpy(msg, "Unknown Error");
      break;
    case VNERR_NOT_IMPLEMENTED:
      strcpy(msg, "Not implemented");
      break;
    case VNERR_TIMEOUT:
      strcpy(msg, "Timemout");
      break;
    case VNERR_INVALID_VALUE:
      strcpy(msg, "Invalid value");
      break;
    case VNERR_FILE_NOT_FOUND:
      strcpy(msg, "File not found");
      break;
    case VNERR_NOT_CONNECTED:
      strcpy(msg, "Not connected");
      break;
    default:
      strcpy(msg, "Undefined Error");
  }
}

/////////////////////////////////////////////////
int main( int argc, char* argv[] )
{
  // Initialize ROS;
  ros::init(argc, argv, "vectornav");
  ros::NodeHandle n; 
  ros::NodeHandle n_("~");
  
  // Read Parameters
  std::string port;
  int baud, poll_rate, async_output_type, async_output_rate;
  
  n_.param<std::string>("serial_port" , port     , "/dev/ttyUSB0");
  n_.param<int>(        "serial_baud" , baud     , 115200);
  n_.param<int>(        "poll_rate"   , poll_rate, 40);
  
  n_.param<std::string>("imu/frame_id", imu_frame_id, "LLA");
  n_.param<std::string>("gps/frame_id", gps_frame_id, "LLA");
   
  // Type: 0 None, 19 IMU, 20 GPS, 22 INS
  n_.param<int>(        "async_output_type"  , async_output_type, 0);
  n_.param<int>(        "async_output_rate"  , async_output_rate, 50); 
  
  // Initialize Publishers
  pub_sensors = n_.advertise<vectornav::sensors>("imu", 1000);
  
  // Initialize VectorNav
  VN_ERROR_CODE vn_retval;
  char vn_error_msg[100];
  ROS_INFO("Initializing vn100. Port:%s Baud:%d\n", port.c_str(), baud);
	vn_retval = vn100_connect(&vn100, port.c_str(), baud);
	if (vn_retval != VNERR_NO_ERROR)
  {
    vnerr_msg(vn_retval, vn_error_msg);
	  ROS_FATAL( "Could not connect to device via: %s, Error Text: %s", port.c_str(), vn_error_msg);
	  exit (EXIT_FAILURE);
	}
	
  vn_retval = vn100_setAsynchronousDataOutputType(&vn100, async_output_type, true);
	if (vn_retval != VNERR_NO_ERROR)
  {
    vnerr_msg(vn_retval, vn_error_msg);
	  ROS_FATAL( "Could not set output type on device via: %s, Error Text: %s", port.c_str(), vn_error_msg);
	  exit (EXIT_FAILURE);
	}
	
  ros::Timer poll_timer; 
	if (async_output_type == 0)
	{
	  // Polling loop
    ROS_INFO("Polling at %d Hz\n", poll_rate);
	  poll_timer = n.createTimer(ros::Duration(1.0/(double)poll_rate), poll_timerCB);
  }
  else
  {
    // Async Request
    switch(async_output_rate)
    {
      case 1:
      case 2:
      case 4:
      case 5:
      case 10:
      case 20:
      case 25:
      case 40:
      case 50:
      case 100:
      case 200:
        ROS_INFO("Establishing an ASYNC subscription at %d Hz\n", async_output_rate);
        break;
      default:
        ROS_ERROR("Invalid ASYNC rate specified (%d). "
                  "Valid rates: {1,2,4,5,10,25,40,50,100,200}", async_output_rate);
    }
    vn100_setAsynchronousDataOutputFrequency(&vn100, async_output_rate, true);
	  vn100_registerAsyncDataReceivedListener(&vn100, &asyncDataListener);
	}
	
  ros::spin();
	
  vn100_disconnect(&vn100);
  return 0;
}
