/*****************************************************************************
 *
 *  Copyright (C) 2013, NIPPON CONTROL SYSTEM Corporation
 *  All rights reserved.
 *  
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   
 *    * Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *   
 *    * Neither the name of the NIPPON CONTROL SYSTEM Corporation nor
 *      the names of its contributors may be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *  
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include "fx8_driver_nodelet.h"
#include <signal.h>

namespace infinisoleil
{
FX8DriverNodelet::FX8DriverNodelet()
  :device_(FX8_INVALID_HANDLE),
   diagnostics_enable_(true),
   device_running_(false),
   target_frequency_(0.0)
{
  scan_.xy_data.surface_count = 0 ;
  scan_.xy_data.surface = NULL;
  memset(&config_, 0, sizeof(Config));
}

void FX8DriverNodelet::onInit()
{
  initializeNodelet();

  driver_thread_ = boost::thread(boost::bind(&FX8DriverNodelet::driverThreadFunc, this));
}

FX8DriverNodelet::~FX8DriverNodelet()
{
  device_running_ = false;

  /*
   *  First, shut down reconfigure server. It prevents destructor of
   *  reconfigure server from stopping. It occurs often at killing
   *  nodelet manager and FX8DriverNodelet by SIGINT when these are
   *  launched by same launch file. And, this problem seems to occur
   *  at unloading FX8DriverNodelet by request of unloading it before
   *  shutting down nodelet manager.
   */
  shutdownReconfigureServer();

  driver_thread_.join();

  shutdownNodelet();
}

void FX8DriverNodelet::initializeNodelet()
{
  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle param_nh = getPrivateNodeHandle();
 
  // Initialize publisher.
  range_publisher_ = nh.advertise<sensor_msgs::Image>("range_image", 1000);
  ir_publisher_ = nh.advertise<sensor_msgs::Image>("ir_image", 1000);
  point_cloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000);

  // Get frame id of topics.
  param_nh.param("range_frame_id", scan_.range_frame_id, std::string("range_image"));
  param_nh.param("ir_frame_id", scan_.ir_frame_id, std::string("ir_image"));
  param_nh.param("point_cloud_frame_id", scan_.point_cloud_frame_id, std::string("point_cloud"));

  // Setup dynamic reconfigure server.
  initializeReconfigureServer(param_nh);

  // Read flag of diagnostics_enable.
  param_nh.param("diagnostics_enable", diagnostics_enable_, true);
  if (diagnostics_enable_)
    setupDiagnostics();
}

void FX8DriverNodelet::shutdownNodelet()
{
  ros::NodeHandle param_nh = getPrivateNodeHandle();

  // Shut down publisher.
  range_publisher_.shutdown();
  ir_publisher_.shutdown();
  point_cloud_publisher_.shutdown();

  // Shut down diagnostic updater.
  diagnostic_updater_.reset();

  // Remove parameter form parameter server. 
  param_nh.deleteParam("diagnostics_enable");
  param_nh.deleteParam("hostname");
  param_nh.deleteParam("port_number");
  param_nh.deleteParam("connect_timeout");
  param_nh.deleteParam("send_timeout");
  param_nh.deleteParam("receive_timeout");
  param_nh.deleteParam("measure_point_x");
  param_nh.deleteParam("measure_point_y");
  param_nh.deleteParam("swing_fs");
  param_nh.deleteParam("swing_ss");
  param_nh.deleteParam("xy_surface_count");
  param_nh.deleteParam("frame_cycle");
  param_nh.deleteParam("measure_mode");
  param_nh.deleteParam("max_measure_mode");
  param_nh.deleteParam("xy_serial_number");
  param_nh.deleteParam("logic_version");
  param_nh.deleteParam("firm_version");
  param_nh.deleteParam("product_number");
  param_nh.deleteParam("range_frame_id");
  param_nh.deleteParam("ir_frame_id");
  param_nh.deleteParam("point_cloud_frame_id");
}

void FX8DriverNodelet::initializeReconfigureServer(ros::NodeHandle param_nh)
{
  reconfigure_server_.reset(new ReconfigureServer(param_nh));
  reconfigure_server_->setCallback(boost::bind(&FX8DriverNodelet::reconfigureFX8Callback, this, _1, _2));
}

void FX8DriverNodelet::shutdownReconfigureServer()
{
  reconfigure_server_.reset();
}

void FX8DriverNodelet::setupDiagnostics()
{
  boost::mutex::scoped_lock lock(mutex_diagnostics_);
  diagnostic_updater_.reset(new diagnostic_updater::Updater());
  diagnostic_updater_->add("Received Error Code from Infinisoleil",
    boost::bind(&FX8DriverNodelet::fillDiagnosticStatusByReceivedErrorCode, this, _1));
  diagnostic_updater_->add("Infinisoleil Error Info",
    boost::bind(&FX8DriverNodelet::fillDiagnosticStatusByErrorInfo, this, _1));

  range_image_diagnostic_frequency_.reset(
    new TopicDiagnostic("range_image", *diagnostic_updater_,
      diagnostic_updater::FrequencyStatusParam(&target_frequency_, &target_frequency_, 0.25)));

  ir_image_diagnostic_frequency_.reset(
    new TopicDiagnostic("ir_image", *diagnostic_updater_,
      diagnostic_updater::FrequencyStatusParam(&target_frequency_, &target_frequency_, 0.25)));

  point_cloud_diagnostic_frequency_.reset(
    new TopicDiagnostic("point_cloud", *diagnostic_updater_,
      diagnostic_updater::FrequencyStatusParam(&target_frequency_, &target_frequency_, 0.25)));
}

void FX8DriverNodelet::driverThreadFunc()
{ 
  bool result = false;
  // Initialize device.
  if (!initializeDevice())
  {
    NODELET_ERROR("Failed to initialize Infinisoleil.");
    // Wait for publishing diagnostics.
    sleep(1);
    goto Exit;
  }

  NODELET_INFO("Succeeded in initializing Infinisoleil.");

  // Start fx8 ranging.
  if (!startScan())
    goto Exit;

  device_running_ = true;
  NODELET_INFO("Start scan.");

  spin();

  result = true;
Exit:
  if (diagnostic_updater_ && diagnostics_enable_ && !result)
  {
    diagnostic_updater_->force_update();
  }

  shutdownDevice();
  return;
}

bool FX8DriverNodelet::initializeDevice()
{
  ros::NodeHandle param_nh = getPrivateNodeHandle();
  FX8Handle device = FX8_INVALID_HANDLE;
  std::string hostname("");
  int port_number = 0;
  int measure_mode = 0;
  int connect_timeout = 0;
  int send_timeout = 0;
  int receive_timeout = 0;
  FX8SensorInfo sensor_info;
  FX8XyData xy_data;
  xy_data.surface_count = 0;
  xy_data.surface = NULL;

  int ret = FX8_ERROR_SUCCESS;
  std::stringstream ss;

  // Get initial parameters.
  param_nh.param("hostname", hostname, std::string("192.168.0.80"));
  param_nh.param("port_number", port_number, 50000);
  param_nh.param("measure_mode", measure_mode, 0);
  param_nh.param("connect_timeout", connect_timeout, 10000);
  if (connect_timeout < 0)
  {
    NODELET_WARN("Failed to set connect timeout. %d is less than 0. Set default value (10000).",
      connect_timeout);
      connect_timeout = 10000;
  }

  param_nh.param("send_timeout", send_timeout, 3000);
  if (send_timeout < 0)
  {
    NODELET_WARN(
      "Failed to set send timeout. %d is less than 0. Set default value (3000).", send_timeout);
      send_timeout = 3000;
  }
  param_nh.param("receive_timeout", receive_timeout, 5000);
  if (receive_timeout < 0)
  {
    NODELET_WARN(
      "Failed to set receive timeout. %d is less than 0. Set default value (5000).", receive_timeout);
      receive_timeout = 5000;
  }

  if (diagnostics_enable_ && diagnostic_updater_)
    diagnostic_updater_->setHardwareIDf("Infinisoleil:[%s:%d]", hostname.c_str(), port_number);

  // Create instance.
  ret = fx8_create_handle(&device);
  if (!device || ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to create Infinisoleil handle.";
    goto Exit;
  }

  // Connect to device.
  ret = fx8_set_connect_timeout(device, connect_timeout);
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to set connect timeout.";
    goto Exit;
  }
  connect_timeout = fx8_get_connect_timeout(device);

  ret = fx8_connect(device, hostname.c_str(), static_cast<unsigned short>(port_number));
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to connect to Infinisoleil. [" << hostname << ':' << static_cast<unsigned short>(port_number) << ']';
    goto Exit;
  }
  NODELET_INFO("Infinisoleil is connected. [%s:%d]", hostname.c_str(), static_cast<unsigned short>(port_number));

  // Set measure mode.
  if (!setDeviceMeasureMode(device, static_cast<FX8MeasureMode>(measure_mode), &sensor_info, &xy_data))
  {
    goto Exit;
  }

  // Set timeout.
  ret = fx8_set_send_timeout(device, send_timeout);
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to set send timeout.";
    goto Exit;
  }
  send_timeout = fx8_get_send_timeout(device);
  ret = fx8_set_receive_timeout(device, receive_timeout);
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to set receive timeout.";
    goto Exit;
  }
  receive_timeout = fx8_get_receive_timeout(device);

  // Set device data.
  device_ = device;
  memcpy(&config_.sensor_info, &sensor_info, sizeof(FX8SensorInfo));
  scan_.xy_data.surface_count = xy_data.surface_count;
  scan_.xy_data.surface = xy_data.surface;
  target_frequency_ = 1 / (config_.sensor_info.frame_cycle / 1000.0);
  if (diagnostics_enable_ && diagnostic_updater_)
  {
    unsigned char* product_number = config_.sensor_info.product_number;
    diagnostic_updater_->setHardwareIDf("%c%c%c%c%c%c%c%c", product_number[0], product_number[1], product_number[2],
      product_number[3], product_number[4], product_number[5], product_number[6], product_number[7]);
   }

  // Write parameters to parameter server.
  param_nh.setParam("hostname", hostname);
  param_nh.setParam("port_number", port_number);
  param_nh.setParam("connect_timeout", connect_timeout);
  param_nh.setParam("send_timeout", send_timeout);
  param_nh.setParam("receive_timeout", receive_timeout);
  param_nh.setParam("diagnostics_enable", diagnostics_enable_);
  param_nh.setParam("range_frame_id", scan_.range_frame_id);
  param_nh.setParam("ir_frame_id", scan_.ir_frame_id);
  param_nh.setParam("point_cloud_frame_id", scan_.point_cloud_frame_id);
  outputDeviceParameters();

  return true;

Exit:
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << " [return code = " << std::showbase <<  std::hex << std::setw(10) << std::internal 
      << std::setfill('0') << ret << ']';
    NODELET_ERROR("%s", ss.str().c_str());
  }
  {
    boost::mutex::scoped_lock lock(mutex_diagnostics_);
    error_info_.push_back(ErrorInfo(ros::Time::now(), ss.str()));
  }

  if (xy_data.surface != NULL)
    fx8_free_xy_data(&xy_data);

  return false;
}

void FX8DriverNodelet::shutdownDevice()
{
  if (device_)
  {
    fx8_stop_ranging(device_);
    fx8_disconnect(device_);
    fx8_close_handle(device_);
    device_ = FX8_INVALID_HANDLE;
    if (scan_.xy_data.surface != NULL)
      fx8_free_xy_data(&scan_.xy_data);
    NODELET_DEBUG("Infinisoleil closed.");
  }
}

bool FX8DriverNodelet::setDeviceMeasureMode(const FX8Handle device, const FX8MeasureMode measure_mode,
  FX8SensorInfo* sensor_info, FX8XyData* xy_data)
{
  FX8XyData current_xy_data;
  current_xy_data.surface_count = 0;
  current_xy_data.surface = NULL;
  FX8SensorInfo current_sensor_info;

  // Get current FX8SensorInfo.
  if (!getDeviceSensorInfo(device, &current_sensor_info, &current_xy_data))
    return false;

  int ret = fx8_set_measure_mode(device, measure_mode);
  if (ret != FX8_ERROR_SUCCESS)
  {
    std::stringstream ss;
    ss << "Failed to set measure mode. (Measure mode[0 - "
       << static_cast<unsigned int>(current_sensor_info.max_measure_mode) << "])";
    ss << " [return code = " << std::showbase <<  std::hex << std::setw(10) << std::internal 
     << std::setfill('0') << ret << ']';
    NODELET_ERROR("%s", ss.str().c_str());
    {
      boost::mutex::scoped_lock lock(mutex_diagnostics_);
      error_info_.push_back(ErrorInfo(ros::Time::now(), ss.str()));
    }
    if (current_xy_data.surface != NULL)
      fx8_free_xy_data(&current_xy_data);
    return false;
  }

  if (current_xy_data.surface != NULL)
    fx8_free_xy_data(&current_xy_data);

  // Get new FX8SensorInfo.
  if (!getDeviceSensorInfo(device, sensor_info, xy_data))
    return false;

  return true;
}

bool FX8DriverNodelet::getDeviceSensorInfo(const FX8Handle device, FX8SensorInfo* sensor_info, FX8XyData* xy_data)
{
  // Set FX8 properties.
  int ret = fx8_get_sensor_property(device, NULL, sensor_info, xy_data);
  if (ret != FX8_ERROR_SUCCESS)
  {
    std::stringstream ss;
    ss << "Failed to get current sensor info.";
    ss << " [return code = " << std::showbase <<  std::hex << std::setw(10) << std::internal 
     << std::setfill('0') << ret << ']';
    NODELET_ERROR("%s", ss.str().c_str());
    {
      boost::mutex::scoped_lock lock(mutex_diagnostics_);
      error_info_.push_back(ErrorInfo(ros::Time::now(), ss.str()));
    }
    return false;
  }

  return true;
}

void FX8DriverNodelet::outputDeviceParameters()
{
  char buf[128];
  unsigned char* xy_serial_number = config_.sensor_info.xy_serial_number;
  unsigned char* firm_version = config_.sensor_info.firm_version;
  unsigned char* logic_version = config_.sensor_info.logic_version;
  unsigned char* product_number = config_.sensor_info.product_number;

  ros::NodeHandle param_nh = getPrivateNodeHandle();

  // Update parameter server.
  param_nh.setParam("measure_point_x", config_.sensor_info.measure_point.x);
  param_nh.setParam("measure_point_y", config_.sensor_info.measure_point.y);
  param_nh.setParam("swing_fs", config_.sensor_info.swing_fs);
  param_nh.setParam("swing_ss", config_.sensor_info.swing_ss);
  param_nh.setParam("xy_surface_count", config_.sensor_info.xy_surface_count);
  param_nh.setParam("frame_cycle", config_.sensor_info.frame_cycle);
  param_nh.setParam("measure_mode", config_.sensor_info.measure_mode);
  param_nh.setParam("max_measure_mode", config_.sensor_info.max_measure_mode);

  sprintf(buf, "%02X.%02X.%02X.%02X", xy_serial_number[0], xy_serial_number[1], xy_serial_number[2],
     xy_serial_number[3]);
  param_nh.setParam("xy_serial_number", buf);

  sprintf(buf, "%u.%u.%u", logic_version[0], logic_version[1], logic_version[2]);
  param_nh.setParam("logic_version", buf);

  sprintf(buf, "%u.%u.%u", firm_version[0], firm_version[1], firm_version[2]);
  param_nh.setParam("firm_version", buf);

  sprintf(buf, "%c%c%c%c%c%c%c%c", product_number[0], product_number[1], product_number[2], product_number[3],
    product_number[4], product_number[5], product_number[6], product_number[7]);
  param_nh.setParam("product_number", buf);
  
  return;
}

bool FX8DriverNodelet::startScan()
{
  int ret = FX8_ERROR_SUCCESS;
  std::stringstream ss;
  // Start waiting for scan data of FX8.
  ret = fx8_set_receive_range_data_event_handler(device_, receiveScanDataCallback, reinterpret_cast<void*>(this));
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to set receive range data event handler.";
    goto Exit;
  }
  ret = fx8_set_receive_error_info_event_handler(device_, receiveErrorCodeCallback, reinterpret_cast<void*>(this));
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to set receive error info event handler.";
    goto Exit;
  }

  ret = fx8_start_ranging(device_);
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to start ranging.";
    goto Exit;
  }
  
  return true;

Exit:
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << " [return code = " << std::showbase <<  std::hex << std::setw(10) << std::internal 
      << std::setfill('0') << ret << ']';
    NODELET_ERROR("%s", ss.str().c_str());
    {
      boost::mutex::scoped_lock lock(mutex_diagnostics_);
      error_info_.push_back(ErrorInfo(ros::Time::now(), ss.str()));
    }
  }
  return false;
}

void FX8DriverNodelet::spin()
{
  while (device_running_ && ros::ok())
  {
    if (diagnostics_enable_ && diagnostic_updater_)
      diagnostic_updater_->update();

    boost::this_thread::sleep(boost::posix_time::milliseconds(5));
  }
}

void FX8DriverNodelet::publishScanData(const unsigned char* scan_data, size_t size)
{
  if (!device_running_)
    return;

  // Lock scan.
  boost::mutex::scoped_lock lock(mutex_scan_);
  
  // Obtained scan data size.
  size_t header_size = 11;
  size_t internal_data_size = 64;
  size_t obtained_data_size = size - header_size - internal_data_size;

  // Calculated scan data size by FX8SensorInfo.
  size_t scan_data_size_per_point = 3; // bytes
  size_t calculated_data_size = config_.sensor_info.measure_point.x
    * config_.sensor_info.measure_point.y * scan_data_size_per_point;
  if (obtained_data_size != calculated_data_size)
  {
    NODELET_ERROR("Obtained data is not data of current measure mode.");
    return;
  }
 
  scan_.scan_data.assign(scan_data, &scan_data[size]);

  // Compare scan data
  unsigned char surf = scan_.scan_data[6];
  if (scan_.xy_data.surface_count <= static_cast<int>(surf))
  {
    NODELET_ERROR("Publish no message. Surface number is out of range.");
    return;
  }

  // Create subscribed messages.
  sensor_msgs::ImagePtr range_msg = createRangeImageMessage();
  sensor_msgs::ImagePtr ir_msg = createIRImageMessage();
  sensor_msgs::PointCloud2Ptr point_cloud_msg = createPointCloudMessage();

  // Return, if subscribed message is nothing.
  if (!range_msg && !ir_msg && !point_cloud_msg)
  {
    NODELET_DEBUG("Publish no messages. Subscribed topic is nothing.");
    return;
  }

  // Set publish data.
  setMessageData(range_msg, ir_msg, point_cloud_msg, surf);

  // Publish message.
  if (range_msg)
  {
    range_publisher_.publish(range_msg);
    if (diagnostics_enable_)
      range_image_diagnostic_frequency_->tick();
    NODELET_DEBUG("Publish range_image.");
  }
  if (ir_msg)
  {
    ir_publisher_.publish(ir_msg);
    if (diagnostics_enable_)
      ir_image_diagnostic_frequency_->tick();
    NODELET_DEBUG("Publish ir_image.");
  }
  if (point_cloud_msg)
  {
    point_cloud_publisher_.publish(point_cloud_msg);
    if (diagnostics_enable_)
      point_cloud_diagnostic_frequency_->tick();
    NODELET_DEBUG("Publish point_cloud.");
  }
}

sensor_msgs::ImagePtr FX8DriverNodelet::createRangeImageMessage()
{
  sensor_msgs::ImagePtr msg;
  if (range_publisher_.getNumSubscribers() <= 0)
    return msg;

  msg.reset(new sensor_msgs::Image());
  msg->header.stamp = latest_update_time_;
  msg->header.frame_id = scan_.range_frame_id;
  msg->height = config_.sensor_info.measure_point.y;
  msg->width = config_.sensor_info.measure_point.x;
  msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  msg->step = sizeof(unsigned short);
  msg->data.resize(msg->height * msg->width * msg->step);

  return msg;
}

sensor_msgs::ImagePtr FX8DriverNodelet::createIRImageMessage()
{
  sensor_msgs::ImagePtr msg;
  if (ir_publisher_.getNumSubscribers() <= 0)
    return msg;

  msg.reset(new sensor_msgs::Image());
  msg->header.stamp = latest_update_time_;
  msg->header.frame_id = scan_.ir_frame_id;
  msg->height = config_.sensor_info.measure_point.y;
  msg->width = config_.sensor_info.measure_point.x;
  msg->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
  msg->step = sizeof(unsigned short);
  msg->data.resize(msg->height * msg->width * msg->step);

  return msg;
}

sensor_msgs::PointCloud2Ptr FX8DriverNodelet::createPointCloudMessage()
{
  sensor_msgs::PointCloud2Ptr msg;
  if (point_cloud_publisher_.getNumSubscribers() <= 0)
    return msg;

  msg.reset(new sensor_msgs::PointCloud2());
  msg->header.stamp = latest_update_time_;
  msg->header.frame_id = scan_.point_cloud_frame_id;
  msg->height = config_.sensor_info.measure_point.y;
  msg->width = config_.sensor_info.measure_point.x;
  msg->fields.resize(3);
  msg->fields[0].name = "x";
  msg->fields[0].offset = 0;
  msg->fields[0].datatype = 7;
  msg->fields[0].count = 1;
  msg->fields[1].name = "y";
  msg->fields[1].offset = 4;
  msg->fields[1].datatype = 7;
  msg->fields[1].count = 1;
  msg->fields[2].name = "z";
  msg->fields[2].offset = 8;
  msg->fields[2].datatype = 7;
  msg->fields[2].count = 1;
  msg->point_step = sizeof(float) * msg->fields.size();
  msg->row_step = msg->width * msg->point_step;
  msg->data.resize(msg->height * msg->row_step);
  msg->is_dense = false;

  return msg;
}

void FX8DriverNodelet::setMessageData(
  sensor_msgs::ImagePtr range_msg, sensor_msgs::ImagePtr ir_msg, sensor_msgs::PointCloud2Ptr point_cloud_msg,
  unsigned char surface_number)
{
  //Extract publish image and create point cloud from scan data.
  unsigned short* range_data = NULL;
  if (range_msg)
    range_data = reinterpret_cast<unsigned short*>(&range_msg->data[0]);
  unsigned short* ir_data = NULL;
  if (ir_msg)
    ir_data  = reinterpret_cast<unsigned short*>(&ir_msg->data[0]);
  float* point_cloud_data = NULL;
  if (point_cloud_msg)
    point_cloud_data = reinterpret_cast<float*>(&point_cloud_msg->data[0]);

  FX8XyDataSurface* surface = &scan_.xy_data.surface[surface_number];
  unsigned short intensity;
  unsigned short range;

  for (int i = 0; i < surface->element_count; ++i)
  {
    FX8XyDataElement* element = &surface->element[i];
    int index = element->y * config_.sensor_info.measure_point.x + element->x;
    
    extractRangeAndIntensityFromScanData(i, &scan_.scan_data[0], &range, &intensity);

    if (ir_data)
      ir_data[index] = intensity;
     
    if (range_data)
      range_data[index] = range;

    if (point_cloud_data)
    {
      int point_cloud_index = index * 3;
      point_cloud_data[point_cloud_index] = static_cast<float>(range * element->bx) / 1000;
      point_cloud_data[point_cloud_index + 1] = static_cast<float>(range * element->by) / 1000;
      point_cloud_data[point_cloud_index + 2] = static_cast<float>(range * element->bz) / 1000;
    }
  }
}

void FX8DriverNodelet::extractRangeAndIntensityFromScanData(int index, const unsigned char* scan_data,
 unsigned short* range, unsigned short* intensity)
{
  // 9 bytes are offset to skip header of scan data packets.
  unsigned int scan_data_offset = 9;

  /*
   *  A scan data is a set of range and intensity data and each data is 12 bit
   *  aligned data. A scan data is 3 bytes (24 bit) aligned data.
  */ 
  unsigned int scan_data_size = 3;

  unsigned int target_index = index * scan_data_size + scan_data_offset;

  // Range data is output in digit. A digit is equal to 4 millimeter.
  unsigned int range_per_digit = 4;

  // Extract intensity data from scan data.
  *intensity = (scan_data[target_index] << 4) | ((scan_data[target_index + 1] & 0xF0) >> 4);

  // Extract range data from scan data.
  *range = ((scan_data[target_index + 1] & 0x0F) << 8) | (scan_data[target_index + 2]);

  // Convert from digit to millimeter.
  *range *= range_per_digit;
}

void FX8DriverNodelet::fillDiagnosticStatusByErrorInfo(diagnostic_updater::DiagnosticStatusWrapper &status)
{
  boost::mutex::scoped_lock lock(mutex_diagnostics_);
  if (error_info_.empty())
  {
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "No Infinisoleil Error Info");
    return;
  }

  char key_buf[256];

  status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Infinisoleil Error Info");
  while(!error_info_.empty())
  {
    std::vector<ErrorInfo>::iterator error_info_it = error_info_.begin();
    sprintf(key_buf, "%.lf", error_info_it->first.toSec());
    status.add(key_buf, error_info_it->second.c_str()); 
    NODELET_DEBUG("Infinisoleil Error Information.[%s:%s]", key_buf, error_info_it->second.c_str());
    error_info_.erase(error_info_it);
  }
}

void FX8DriverNodelet::fillDiagnosticStatusByReceivedErrorCode(
  diagnostic_updater::DiagnosticStatusWrapper &status)
{
  boost::mutex::scoped_lock lock(mutex_diagnostics_);
  if (error_code_.empty())
  {
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "No Infinisoleil Received Error Code");
    return;
  }

  char key_buf[256];
  char value_buf[256];
  bool is_warn = true;

  while (!error_code_.empty())
  {
    std::vector<ReceivedErrorCodePackets>::iterator error_code_it = error_code_.begin();
    unsigned char* data = &(error_code_it->second[0]);

    // Set error code.
    unsigned int condition = (data[6] << 8) | data[7];
    unsigned int error_code[6];
    error_code[0] = static_cast<unsigned int>(data[8]);
    error_code[1] = static_cast<unsigned int>(data[9]);
    error_code[2] = static_cast<unsigned int>(data[10]);
    error_code[3] = static_cast<unsigned int>(data[11]);
    error_code[4] = static_cast<unsigned int>(data[12]);
    error_code[5] = static_cast<unsigned int>(data[13]);
    sprintf(value_buf,
      "Error:0x%04X, Code(0):0x%02X, Code(1):0x%02X, Code(2):0x%02X, Code(3):0x%02X, Code(4):0x%02X, Code(5):0x%02X",
      condition, error_code[0], error_code[1], error_code[2], error_code[3], error_code[4], error_code[5]);
    sprintf(key_buf, "%.6lf", error_code_it->first.toSec());
    status.add(key_buf, value_buf);

    if (condition & 0xC000)
    {
      NODELET_WARN("Received Infinisoleil Error Code.[%s:%s]", key_buf, value_buf);
    }
    else
    {
      NODELET_ERROR("Received Infinisoleil Error Code.[%s:%s]", key_buf, value_buf);
      is_warn = false;
    }
    error_code_.erase(error_code_it);
  }

  if (is_warn)
  {
    status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "Infinisoleil Received Error Code");
  }
  else
  {
    status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "Infinisoleil Received Error Code");
  }
}

void FX8DriverNodelet::addDiagnostics(const unsigned char* error_data, size_t size)
{
  // Lock diagnostics.
  boost::mutex::scoped_lock lock(mutex_diagnostics_);

  // Copy error data.
  error_code_.push_back(ReceivedErrorCodePackets(ros::Time::now(), std::vector<unsigned char>()));
  error_code_.back().second.assign(error_data, &error_data[size]);
}

void FX8DriverNodelet::reconfigureFX8Callback(FX8Config config, uint32_t level)
{ 
  int ret = FX8_ERROR_SUCCESS;
  std::stringstream ss;
  ros::NodeHandle param_nh = getPrivateNodeHandle();

  // Check initialization of FX8.
  if (device_ == FX8_INVALID_HANDLE)
    return;

  // Check connection to FX8.
  if (!fx8_get_connected(device_))
  {
    NODELET_WARN("Failed to reconfigure. Infinisoleil is disconnected.");
    return;
  }

  // Reconfigure measure mode of FX8.
  FX8MeasureMode measure_mode = static_cast<FX8MeasureMode>(config.measure_mode);
  if (measure_mode == config_.sensor_info.measure_mode)
  {
    NODELET_INFO("No need reconfiguration. Current measure_mode is %d.", measure_mode);
    return;
  }

  ret = fx8_stop_ranging(device_);
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to stop ranging.";
    ss << " [return code = " << std::showbase <<  std::hex << std::setw(10) << std::internal 
     << std::setfill('0') << ret << ']';    
    NODELET_ERROR("%s", ss.str().c_str());
    {
      boost::mutex::scoped_lock lock(mutex_diagnostics_);
      error_info_.push_back(ErrorInfo(ros::Time::now(), ss.str()));
    }
    return;
  }

  FX8SensorInfo sensor_info;
  FX8XyData xy_data;
  xy_data.surface_count = 0;
  xy_data.surface = NULL;
  // Lock scan.
  {
    boost::mutex::scoped_lock lock(mutex_scan_);

    // Set FX8 measure mode.
    if (!setDeviceMeasureMode(device_, measure_mode, &sensor_info, &xy_data))
    {
      if (xy_data.surface != NULL)
        fx8_free_xy_data(&xy_data);
      return;
    } 
   
    if (scan_.xy_data.surface != NULL)
      fx8_free_xy_data(&scan_.xy_data);
    scan_.xy_data.surface = xy_data.surface;
    scan_.xy_data.surface_count = xy_data.surface_count;
    memcpy(&config_.sensor_info, &sensor_info, sizeof(FX8SensorInfo));
  }
  {
      boost::mutex::scoped_lock lock(mutex_diagnostics_);
      target_frequency_ = 1 / (config_.sensor_info.frame_cycle / 1000.0);
  }
  outputDeviceParameters();

  // Restart ranging.
  ret = fx8_start_ranging(device_);
  if (ret != FX8_ERROR_SUCCESS)
  {
    ss << "Failed to restart ranging.";
    ss << " [return code = " << std::showbase <<  std::hex << std::setw(10) << std::internal 
     << std::setfill('0') << ret << ']';
    NODELET_ERROR("%s", ss.str().c_str());
    {
      boost::mutex::scoped_lock lock(mutex_diagnostics_);
      error_info_.push_back(ErrorInfo(ros::Time::now(), ss.str()));
    }
    return; 
  }
  return;
}

void FX8DriverNodelet::updateTime()
{
  latest_update_time_ = ros::Time::now();
}

void FX8DriverNodelet::receiveScanDataCallback(const unsigned char* scan_data, size_t size, void* user_data)
{
  FX8DriverNodelet* driver = reinterpret_cast<FX8DriverNodelet*>(user_data);
  driver->updateTime();
  driver->publishScanData(scan_data, size);
}

void FX8DriverNodelet::receiveErrorCodeCallback(const unsigned char* error_data, size_t size, void* user_data)
{
  FX8DriverNodelet* driver = reinterpret_cast<FX8DriverNodelet*>(user_data);
  driver->addDiagnostics(error_data, size);
}
} // namespace infinisoleil_fx8

PLUGINLIB_EXPORT_CLASS(infinisoleil::FX8DriverNodelet, nodelet::Nodelet)
