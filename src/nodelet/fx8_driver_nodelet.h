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

#ifndef _FX8_DRIVER_H_
#define _FX8_DRIVER_H_

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp>

#include <fx8.h>
#include <infinisoleil/FX8Config.h>

//! Namespace containing driver.
namespace infinisoleil
{
/*!
 *  \class FX8DriverNodelet
 *  \brief FX8 driver nodelet class.
 */
class FX8DriverNodelet : public nodelet::Nodelet
{
  typedef fx8_handle FX8Handle; //!< Device handle.
  typedef fx8_measure_mode FX8MeasureMode; //!< Device measure mode.
  typedef fx8_bool FX8Bool; //!< Boolean of libfx8.
  typedef fx8_sensor_info FX8SensorInfo; //!< Sensor information.
  typedef fx8_xy_data_element FX8XyDataElement; //!< Element of device xy data.
  typedef fx8_xy_data_surface FX8XyDataSurface; //!< Surface of device xy data.
  typedef fx8_xy_data FX8XyData; //!< Device xy data.
  typedef dynamic_reconfigure::Server<infinisoleil::FX8Config> ReconfigureServer; //!< Dynamic reconfigure server.
  typedef std::vector<unsigned char> ScanDataPackets; //!< Received FX8 scan data packets.
  typedef std::pair<ros::Time, std::vector<unsigned char> > ReceivedErrorCodePackets; //!< Received FX8 error code.
  typedef std::pair<ros::Time, std::string> ErrorInfo; //!< Error information.
  typedef diagnostic_updater::HeaderlessTopicDiagnostic TopicDiagnostic; //!< Topic diagnostic.
  typedef boost::shared_ptr<TopicDiagnostic> TopicDiagnosticPtr; //!< Shared pointer of topic diagnostic.

  /*!
   *  \struct FX8Scan
   *  \brief  Struct of fx8 scan data.
   */
  struct FX8Scan
  {
    FX8XyData xy_data;    //!< FX8 xy data.
    ScanDataPackets scan_data; //!< Scan data.
    std::string range_frame_id; //!< Range frame id.
    std::string ir_frame_id; //!< IR frame id.
    std::string point_cloud_frame_id; //!< Point cloud frame id.
  };

  /*!
   *  \struct FX8Info
   *  \brief  Struct of fx8 information.
   */
  struct FX8Info
  {
    FX8SensorInfo sensor_info; //!< Sensor information.
    int connect_timeout; //!< Connect timeout.
    int send_timeout; //!< Send timeout.
    int receive_timeout; //!< Receive timeout.
  };

  typedef FX8Info Config; //!< Configurations of FX8.
  typedef FX8Scan Scan; //!< Scan data of FX8.

public:
  //! \brief Constructer.
  FX8DriverNodelet();
  
  //! \brief Destructor.
  virtual ~FX8DriverNodelet();

private:
  //! \brief Initialize FX8 driver.
  virtual void onInit();

  //! \brief Initialize nodelet.
  void initializeNodelet();

  //! \brief Shut down nodelet.
  void shutdownNodelet();

  /*!
   *  \brief Initialize reconfigure server.
   *  \param[in] param_nh Private node handle of FX8DriverNodelet.
   */
  void initializeReconfigureServer(ros::NodeHandle param_nh);

  //! \brief Shut down reconfigure server.
  void shutdownReconfigureServer();

  //! \brief Setup diagnostics.
  void setupDiagnostics();

  //! \brief FX8 driver thread.
  void driverThreadFunc();

  /*!
   *  \brief Initialize and connect to FX8.
   *  \return True if initialization is succeeded, false if it is not.
   */
  bool initializeDevice();

  //! \brief Shut down device.
  void shutdownDevice();

  /*!
   *  \brief Set measure mode of FX8.
   *  \param[in] device FX8 handle. 
   *  \param[in] measure_mode Measure mode of FX8.
   *  \param[out] sensor_info Sensor information of FX8.
   *  \param[out] xy_data XY data of FX8.
   */
  bool setDeviceMeasureMode(const FX8Handle device, const FX8MeasureMode measure_mode,
  FX8SensorInfo* sensor_info, FX8XyData* xy_data);

  /*!
   *  \brief Get FX8SensorInfo and set these.
   *  \param[in] device FX8 handle.
   *  \param[out] sensor_info Sensor information of FX8.
   *  \param[out] xy_data XY data of FX8.
   *  \return True if FX8SensorInfo is obtained, false if it is not.
   */
  bool getDeviceSensorInfo(const FX8Handle device, FX8SensorInfo* sensor_info, FX8XyData* xy_data);

  //! \brief Output FX8 parameters to parameter server.
  void outputDeviceParameters();

  /*!
   *  \brief Start FX8 scan.
   *  \return True if starting scan is succeeded, false if it is not.
   */  
  bool startScan();

  //! \brief Spin driver thread.
  void spin();

  /*!
   *  \brief Publish scan data.
   *  \param[in] scan_data FX8 scan data.
   *  \param[in] size Size of scan data.
   */
  void publishScanData(const unsigned char* scan_data, size_t size);

  //! \brief Create message for range_image topic.
  sensor_msgs::ImagePtr createRangeImageMessage();

  //! \brief Create message for ir_image topic.
  sensor_msgs::ImagePtr createIRImageMessage();

  //! \brief Create message for point_cloud topic.
  sensor_msgs::PointCloud2Ptr createPointCloudMessage();
 
  /*!
   *  \brief Set data of range image, IR image and point cloud.
   *  \param[in] range_msg Message for range_image topic.
   *  \param[in] ir_msg Message for ir_image topic.
   *  \param[in] point_cloud_msg Message for point_cloud topic.
   *  \param[in] surface_number Surface number of xy data.
   */
  void setMessageData(
    sensor_msgs::ImagePtr range_msg, sensor_msgs::ImagePtr ir_msg, sensor_msgs::PointCloud2Ptr point_cloud_msg,
    unsigned char surface_number);

  /*!
   *  \brief Extract range and intensity data from scan data packets.
   *  \param[in] index Index of scan data packets.
   *  \param[in] scan_data Scan data.
   *  \param[out] range Range data.
   *  \param[out] intensity Intensity data.
   */
  inline void extractRangeAndIntensityFromScanData(int index, const unsigned char* scan_data,
    unsigned short* range, unsigned short* intensity);

  /*!
   *  \brief Fill diagnostic status by error information of FX8.
   *  \param[in] status Diagnostic status of FX8.
   */
  void fillDiagnosticStatusByErrorInfo(diagnostic_updater::DiagnosticStatusWrapper &status);

  /*!
   *  \brief Fill diagnostic status by received error code of FX8.
   *  \param[in] status Diagnostic status of FX8.
   */
  void fillDiagnosticStatusByReceivedErrorCode(diagnostic_updater::DiagnosticStatusWrapper &status);

  /*!
   *  \brief Add obtained error data.
   *  \param[in] error_data Error data from FX8.
   *  \param[in] size Size of error data.
   */
  void addDiagnostics(const unsigned char* error_data, size_t size);

  /*!
   *  \brief Reconfigure FX8.
   *  \param[in] config Parameters for reconfiguration.
   *  \param[in] level Bitmask for changed parameters.
   */
  void reconfigureFX8Callback(FX8Config config, uint32_t level);

  //! \brief Update timestamp.
  inline void updateTime();

  /*!
   *  \brief Receive FX8 scan data.
   *  \param[in] scan_data Scan data.
   *  \param[in] size Size of scan data.
   *  \param[in] user_data User-defined data.
   */
  static void receiveScanDataCallback(const unsigned char* scan_data, size_t size, void* user_data);

  /*!
   *  \brief Receive FX8 Error code.
   *  \param[in] error_data Error data.
   *  \param[in] size Size of error data.
   *  \param[in] user_data User-defined data.
   */
  static void receiveErrorCodeCallback(const unsigned char* error_data, size_t size, void* user_data);

  // Device manager
  FX8Handle device_; //!< Handle of FX8.
  Scan scan_; //!< Scan data from FX8.
  Config config_; //!< Configurations of FX8.
  std::vector<ReceivedErrorCodePackets> error_code_; //!< Received error code from FX8.
  std::vector<ErrorInfo> error_info_; //!< Error information of FX8.

  boost::thread driver_thread_; //!< FX8 driver thread.
  bool device_running_; //!< Flag of running device.

  // Publisher
  ros::Publisher range_publisher_; //!< Publisher of range image.
  ros::Publisher ir_publisher_; //!< Publisher of IR image.
  ros::Publisher point_cloud_publisher_; //!< Publisher of point cloud.
  boost::mutex mutex_scan_; //!< Mutex for scan data.
  ros::Time latest_update_time_; //!< Timestamp for topics.

  // Diagnostic
  boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_; //!< Diagnostic updater.
  TopicDiagnosticPtr range_image_diagnostic_frequency_; //!< Topic diagnostic for range image.
  TopicDiagnosticPtr ir_image_diagnostic_frequency_; //!< Topic diagnostic for IR image.
  TopicDiagnosticPtr point_cloud_diagnostic_frequency_; //!< Topic diagnostic for point cloud.
  boost::mutex mutex_diagnostics_; //!< Mutex for diagnostics.
  bool diagnostics_enable_; //!< Flag to enable diagnostics.
  double target_frequency_; //!< Target frame rate of topics.

  // Reconfig
  boost::shared_ptr<ReconfigureServer> reconfigure_server_; //!< Server for reconfiguration.
};
} // namespace infinisoleil

#endif // _FX8_DRIVER_H_
