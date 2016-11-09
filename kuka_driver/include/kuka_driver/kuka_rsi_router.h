#ifndef KUKA_RSI_rsi_server_H_
#define KUKA_RSI_rsi_server_H_

#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <tinyxml.h>

#include <kuka_driver/udp_server.h>

class KUKARSIRouter
{
public:
  KUKARSIRouter() :
  cart_actual_pos_(6, 0),
  cart_setpoint_pos_(6, 0),
  joint_actual_pos_(6, 0),
  joint_setpoint_pos_(6, 0),
  joint_position_correction_(6,0),
  cart_correction_(6,0),
  ipoc_(0),
  is_joint_position_correction_updated_(false),
  is_rsi_in_updated_(false),
  do_serve_rsi_(false),
  do_serve_ext_control_(false),
  late_external_control_packages_(0),
  max_late_external_control_packages_(3),
  is_ext_control_server_connected_(false),
  rsi_server_(udp::endpoint(udp::v4(), 49152)),
  ext_control_server_(udp::endpoint(udp::v4(), 10000))
  {

  }

  ~KUKARSIRouter()
  {

  }

  std::vector<double> cart_actual_pos()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cart_actual_pos_;
  }

  std::vector<double> cart_setpoint_pos()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return cart_setpoint_pos_;
  }

  std::vector<double> joint_actual_pos()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_actual_pos_;
  }

  std::vector<double> joint_setpoint_pos()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return joint_setpoint_pos_;
  }

  void set_joint_position_correction(std::vector<double> joint_position_correction)
  {
    std::lock_guard<std::mutex> lock(joint_position_correction_mutex_);
    joint_position_correction_ = joint_position_correction;
  }

  std::vector<double> joint_position_correction()
  {
    std::lock_guard<std::mutex> lock(joint_position_correction_mutex_);
    return joint_position_correction_;
  }

  //void set_cart_correction(std::vector<double> cart_correction)
  //{
    //std::lock_guard<std::mutex> lock(mutex_);
    //cart_correction_ = cart_correction;
    //is_updated_ = true;
  //}

  unsigned long long ipoc()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ipoc_;
  }

  // start and stop rsi server thread
  bool startRSIServer()
  {
    std::cout << "Starting RSI server" << std::endl;
    if (!do_serve_rsi_)
    {
      do_serve_rsi_ = true;
      rsi_server_worker_thread_ = std::thread(&KUKARSIRouter::serveRSI, this);
      return true;
    }
    else
    {
      // cannot start an already started thread
      return false;
    }
  }

  bool stopRSIServer()
  {
    if (do_serve_rsi_)
    {
      do_serve_rsi_ = false;
      rsi_server_worker_thread_.join();
      return true;
    }
    else
    {
      // cannot stop a non-running thread
      return false;
    }
  }

  bool startExtControlServer()
  {
      while (true)
      {

    is_ext_control_server_connected_ = ext_control_server_.waitForConnection("RSI");
    if (!do_serve_ext_control_)
    {
      do_serve_ext_control_ = true;
      std::cout << "Connected to external control client: " << ext_control_server_.remote_endpoint() << std::endl;
      ext_control_server_.send<std::string>("you have now control over the KUKA robot");
      ext_control_worker_thread_ = std::thread(&KUKARSIRouter::serveExtControl, this);
      ext_control_worker_thread_.join();
      std::cout << "Disconnecting external control client" << std::endl;
      do_serve_ext_control_ = false;
    }
      }
  }

  //bool stopExtControlServer()
  //{

  //}


protected:
  // xml functions
  void parseXML(std::string);
  std::string createXML(std::vector<double> joint_position_correction, unsigned long long ipoc);

  // worker thread
  void serveRSI();
  void serveExtControl();

private:

  UDPServer rsi_server_;
  UDPServer ext_control_server_;

  bool is_ext_control_server_connected_;

  bool do_serve_rsi_;
  std::thread rsi_server_worker_thread_;

  bool do_serve_ext_control_;
  std::thread ext_control_worker_thread_;

  std::mutex mutex_;
  std::mutex joint_position_correction_mutex_;

  std::condition_variable joint_position_correction_updated_cv_;
  std::condition_variable cv_;

  bool is_joint_position_correction_updated_;
  bool is_rsi_in_updated_;

  int late_external_control_packages_;
  int max_late_external_control_packages_;

  std::vector<double> cart_actual_pos_;
  std::vector<double> cart_setpoint_pos_;
  std::vector<double> joint_actual_pos_;
  std::vector<double> joint_setpoint_pos_;
  std::vector<double> joint_position_correction_;
  std::vector<double> cart_correction_;
  // ipoc timestamp
  unsigned long long ipoc_;


};

#endif
