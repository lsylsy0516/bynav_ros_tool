////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/callback_queue.h>

#include <mutex>
#include <condition_variable>
#include <map>
#include <thread>

#include <boost/asio.hpp>

#include "novatel_oem7_msgs/Oem7AbasciiCmd.h"
#include "novatel_oem7_msgs/Oem7RawMsg.h"

#include <pluginlib/class_loader.h>

#include <bynav_ros_driver/oem7_receiver_if.hpp>
#include <bynav_ros_driver/oem7_message_decoder_if.hpp>

#include <bynav_ros_driver/oem7_message_util.hpp>
#include <bynav_ros_driver/ros_messages.hpp>
#include <oem7_ros_publisher.hpp>

#include <message_handler.hpp>

#include "bynav_ros_driver/tcp_client.hpp"

namespace {

void genBaseEncode(char *base64_str, const unsigned char *byte, int32_t n) {
  const char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int i, j, k, b;

  for (i = j = 0; i / 8 < n;) {
    for (k = b = 0; k < 6; ++k, ++i) {
      b <<= 1;
      if (i / 8 < n) {
        b |= (byte[i / 8] >> (7 - i % 8)) & 0x1;
      }
    }
    base64_str[j++] = table[b];
  }

  while (j & 0x3) {
    base64_str[j++] = '=';
  }
  base64_str[j] = '\0';
}

std::string genNtripAuthorizeStr(const std::string& username,
                                 const std::string& passwd,
                                 const std::string& mount_point) {
  char enc[512] = {0};
  char base64_str[512] = {0};
  if (username.size() > 128 || passwd.size() > 256) {
    return "";
  }

  sprintf(enc, "%s:%s", username.c_str(), passwd.c_str());
  genBaseEncode(base64_str, (const unsigned char *)enc, strlen(enc));

  char buff[1024];
  sprintf(buff,
          "GET /%s HTTP/1.0\r\n"
          "User-Agent: NTRIP connect\r\n"
          "Authorization: Basic %s\r\n\r\n",
          mount_point.c_str(), base64_str);

  return std::string(buff);
}

constexpr auto kUpdateGap = 20;  // s

}

namespace bynav_ros_driver
{
  /**
   * Nodelet publishing raw oem7 messages and issuing oem7 commands.
   * Loads plugins responsible for obtaining byte input from the Oem7 receiver, and decoding it into raw oem7 messages.
   * Implements a service allowing Oem7 Abbreviated ASCII commands to be sent to the receiver.
   */
  class Oem7MessageNodelet :
      public Oem7MessageDecoderUserIf,
      public nodelet::Nodelet
  {
    std::mutex nodelet_mtx_; ///< Protects nodelet internal state

    double publish_delay_sec_; ///< Delay after publishing each message; used to throttle output with static data sources.

    Oem7RosPublisher oem7rawmsg_pub_; ///< Publishes raw Oem7 messages.
    bool publish_unknown_oem7raw_; ///< Publish all unknown messages to 'Oem7Raw'

    ros::CallbackQueue timer_queue_; ///< Dedicated queue for command requests.
    boost::shared_ptr<ros::AsyncSpinner> timer_spinner_; ///< 1 thread servicing the command queue.

    // Command service
    std::condition_variable rsp_ready_cond_; ///< Response ready, signaled from response handler to Oem7 Cmd Handler.
    std::mutex              rsp_ready_mtx_; ///< Response condition guard
    std::string rsp_; ///< The latest response from Oem7 receiver.
    ros::CallbackQueue queue_; //< Dedicated queue for command requests.
    boost::shared_ptr<ros::AsyncSpinner> aspinner_; ///< 1 thread servicing the command queue.
    ros::ServiceServer oem7_cmd_srv_; ///< Oem7 command service.


    ros::Timer timer_; ///< One time service callback.

    pluginlib::ClassLoader<bynav_ros_driver::Oem7ReceiverIf> recvr_loader_;
    pluginlib::ClassLoader<bynav_ros_driver::Oem7MessageDecoderIf> oem7_msg_decoder_loader;

    std::set<int> raw_msg_pub_; ///< Set of raw messages to publish.

    boost::shared_ptr<MessageHandler> msg_handler_; ///< Dispatches individual messages for handling.

    // Log statistics
    long total_log_count_; ///< Total number of logs received

    typedef std::map<int, long> log_count_map_t;
    log_count_map_t log_counts_; ///< Indidividual log counts.
    
    long unknown_msg_num_;   ///< number of messages received that could not be identified.
    long discarded_msg_num_; ///< Number of messages received and discarded by the driver.


    boost::shared_ptr<bynav_ros_driver::Oem7MessageDecoderIf> msg_decoder; ///< Message Decoder plugin
    boost::shared_ptr<bynav_ros_driver::Oem7ReceiverIf> recvr_; ///< Oem7 Receiver Interface plugin

    enum class NtripAuthStatus { kAuthOk = 0, kAuthFailed };

    std::atomic_bool is_running_;
    std::thread rtcm_data_thread_;
    std::string gpgga_data_;
    boost::shared_ptr<TcpClient> ntrip_cli_;
    NtripAuthStatus auth_status_;
    std::chrono::steady_clock::time_point current_time_;
    std::chrono::steady_clock::time_point last_time_;

    std::string getNtripParam(const std::string& key) {
      using ntrip_param_map = std::map<std::string, std::string>;

      ntrip_param_map ntrip_params;
      getPrivateNodeHandle().getParam("NTRIP", ntrip_params);

      if (ntrip_params.find(key) == ntrip_params.end()) {
        return "";
      }

      return ntrip_params[key];
    }

    bool enableNtrip() {
      const std::string& enable_status = getNtripParam("enable");
      return enable_status == "1";
    }

    void initNtripClient() {
      std::string ip = getNtripParam("ip");
      if (ip.empty()) {
        ip = "127.0.0.1";
      }
      std::string port = getNtripParam("port");
      if (port.empty()) {
        port = "8888";
      }

      ntrip_cli_.reset(new TcpClient(ip, stoi(port)));
      ntrip_cli_->SetNonBlockMode();
    }

    void authorizeNtrip() {
      if (auth_status_ == NtripAuthStatus::kAuthOk) {
        return;
      }

      const std::string& username = getNtripParam("username");
      const std::string& password = getNtripParam("password");
      const std::string& mount_point = getNtripParam("mountpoint");

      NODELET_INFO_STREAM("auth info: " << username << ", " << password << ", "
                                        << mount_point);
      std::string auth_str =
          genNtripAuthorizeStr(username, password, mount_point);
      ntrip_cli_->SendData(auth_str);

      sleep(1);

      int8_t auth_retry_cnt = 3;
      while (auth_retry_cnt--) {
        char buf[1024];
        int32_t ret = ntrip_cli_->RecvData(buf, sizeof(buf));
        if (ret == -1) {
          goto fail;
        }

        std::string resp(buf);
        if (resp.find("ICY 200 OK") != std::string::npos) {
          NODELET_INFO_STREAM("auth ok");
          auth_status_ = NtripAuthStatus::kAuthOk;
          return;
        }

        usleep(10000);
      }

    fail:
      auth_status_ = NtripAuthStatus::kAuthFailed;
      NODELET_ERROR_STREAM("auth failed");
    }

    void handleRTCMData() {
      if (!ntrip_cli_) {
        return;
      }

      is_running_.store(true, std::memory_order_relaxed);
      NODELET_INFO_STREAM("handle rtcm start");
      while (is_running_.load(std::memory_order_relaxed)) {
        usleep(1000);

        if (!ntrip_cli_->IsConnected() ||
            ntrip_cli_->HasLostConnection()) {
          auth_status_ = NtripAuthStatus::kAuthFailed;
          int32_t ret = ntrip_cli_->ConnectToServer();
          if (ret == -1) {
            NODELET_ERROR_STREAM("connect to ntrip server failed");
            sleep(2);
            continue;
          }

          authorizeNtrip();
        }

        if (auth_status_ == NtripAuthStatus::kAuthFailed) {
          ntrip_cli_->DisconnectFromServer();
          continue;
        }

        const auto& rtcm_data = ntrip_cli_->ReadAll();
        if (!rtcm_data.empty() && recvr_) {
          recvr_->write(boost::asio::buffer(rtcm_data));
        }
      }

      NODELET_INFO_STREAM("handle rtcm end");
    }

    void extractGPGGAData(Oem7RawMessageIf::ConstPtr raw_msg) {
      std::string gpgga_data(
          raw_msg->getMessageData(0),
          raw_msg->getMessageData(raw_msg->getMessageDataLength()));
      if (gpgga_data.find("GPGGA") == std::string::npos) {
        return;
      }

      current_time_ = std::chrono::steady_clock::now();
      std::chrono::duration<double> time_gap =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              current_time_ - last_time_);

      if (gpgga_data_.empty() || time_gap.count() > kUpdateGap) {
        NODELET_DEBUG_STREAM("GPGGA: " << gpgga_data);
        gpgga_data_ = gpgga_data;
        last_time_ = current_time_;

        if (ntrip_cli_ && ntrip_cli_->IsConnected()) {
          ntrip_cli_->SendData(gpgga_data);
        }
      }
    }

    /**
     * Wraps actual initializating for exception handling.
     */
    void onInit()
    {
      try
      {
        onInitImpl();
      }
      catch(std::exception  const& ex)
      {
        NODELET_FATAL_STREAM("Fatal: " << ex.what());

        ros::shutdown();
      }
    }

    /**
     * Loads plugins, sets up threading/callbacks, advertises messages and services.
     */
    void onInitImpl()
    {
      std::lock_guard<std::mutex> guard(nodelet_mtx_);

      current_time_ = std::chrono::steady_clock::steady_clock::now();
      last_time_ = std::chrono::steady_clock::steady_clock::now();

      initializeOem7MessageUtil(getNodeHandle());

      getNodeHandle().setCallbackQueue(&timer_queue_);

      getPrivateNodeHandle().getParam("oem7_publish_unknown_oem7raw", publish_unknown_oem7raw_);

      getPrivateNodeHandle().getParam("oem7_publish_delay", publish_delay_sec_);
      if(publish_delay_sec_ > 0)
      {
        NODELET_WARN_STREAM("Publish Delay: " << publish_delay_sec_ << " seconds. Is this is a test?");
      }
      // Load plugins

      // Load Oem7Receiver
      std::string oem7_if_name;
      getPrivateNodeHandle().getParam("oem7_if", oem7_if_name);
      recvr_ = recvr_loader_.createInstance(oem7_if_name);
      recvr_->initialize(getPrivateNodeHandle());

      // Load Oem7 Message Decoder
      std::string msg_decoder_name;
      getPrivateNodeHandle().getParam("oem7_msg_decoder", msg_decoder_name);
      msg_decoder = oem7_msg_decoder_loader.createInstance(msg_decoder_name);
      msg_decoder->initialize(getPrivateNodeHandle(), recvr_.get(), this);


      msg_handler_.reset(new MessageHandler(getPrivateNodeHandle()));

      // Oem7 raw messages to publish.
      std::vector<std::string> oem7_raw_msgs;
      bool ok = getPrivateNodeHandle().getParam("oem7_raw_msgs", oem7_raw_msgs);
      for(const auto& msg : oem7_raw_msgs)
      {
        int raw_msg_id = getOem7MessageId(msg);
        if(raw_msg_id == 0)
        {
          NODELET_ERROR_STREAM("Unknown Oem7 message '" << msg );
        }
        else
        {
          NODELET_INFO_STREAM("Oem7 Raw message '" << msg << "' will be published." );
          raw_msg_pub_.insert(raw_msg_id);
        }
      }

      oem7rawmsg_pub_.setup<novatel_oem7_msgs::Oem7RawMsg>("Oem7RawMsg", getPrivateNodeHandle());

      timer_spinner_.reset(new ros::AsyncSpinner(1, &timer_queue_)); //< 1 thread servicing the command queue.
      timer_spinner_->start();

      timer_ =  getNodeHandle().createTimer(ros::Duration(0.0), &Oem7MessageNodelet::serviceLoopCb, this, true);

      aspinner_.reset(new ros::AsyncSpinner(1, &queue_));
      aspinner_->start();

      sleep(0.5); // Allow all threads to start

      ros::AdvertiseServiceOptions ops = ros::AdvertiseServiceOptions::create<novatel_oem7_msgs::Oem7AbasciiCmd>(
                                                            "Oem7Cmd",
                                                            boost::bind(&Oem7MessageNodelet::serviceOem7AbasciiCb, this, _1, _2),
                                                            ros::VoidConstPtr(),
                                                            &queue_);
      oem7_cmd_srv_ = getPrivateNodeHandle().advertiseService(ops);

      if (enableNtrip()) {
        initNtripClient();
        rtcm_data_thread_ = std::thread(&Oem7MessageNodelet::handleRTCMData, this);
      } else {
        NODELET_INFO_STREAM("ntrip not enabled");
      }
    }


    /**
     * Called to request O7AbasciiCmd service
     */
    bool serviceOem7AbasciiCb(novatel_oem7_msgs::Oem7AbasciiCmd::Request& req, novatel_oem7_msgs::Oem7AbasciiCmd::Response& rsp)
    {
      NODELET_DEBUG_STREAM("AACmd: cmd '" << req.cmd << "'");

      // Retry sending the commands. For configuration commands, there is no harm in duplicates.
      for(int attempt = 0;
              attempt < 10;
              attempt++)
      {
        {
          std::lock_guard<std::mutex> lk(rsp_ready_mtx_);
    	  rsp_.clear();
    	}

        recvr_->write(boost::asio::buffer(req.cmd));
        static const std::string NEWLINE("\n");
        recvr_->write(boost::asio::buffer(NEWLINE));

        std::unique_lock<std::mutex> lk(rsp_ready_mtx_);
        if(rsp_ready_cond_.wait_until(lk,
                                      std::chrono::steady_clock::now() + std::chrono::seconds(3)
                                      ) == std::cv_status::no_timeout)
        {
          rsp.rsp = rsp_;
          break;
        }

        NODELET_ERROR_STREAM("Attempt " << attempt << ": timed out waiting for response.");
      }

      if(rsp.rsp == "OK")
      {
        NODELET_INFO_STREAM("AACmd '" << req.cmd << "' : " << "'" << rsp.rsp << "'");
      }
      else
      {
        NODELET_ERROR_STREAM("AACmd '" << req.cmd << "' : " << "'" << rsp.rsp << "'");
      }

      return true;
    }

    /**
     * Outputs Log statistics to ROS console.
     */
    void outputLogStatistics()
    {
      NODELET_INFO("Log Statistics:");
      NODELET_INFO_STREAM("Logs: " << total_log_count_ << "; unknown: "   << unknown_msg_num_
                                                       << "; discarded: " << discarded_msg_num_);

      for(log_count_map_t::iterator itr = log_counts_.begin();
                                    itr != log_counts_.end();
                                    itr++)
      {
        int  id    = itr->first;
        long count = itr->second;

        NODELET_INFO_STREAM("Log[" << getOem7MessageName(id) << "](" << id << "):" <<  count);
      }
    }

    /*
     * Update Log statistics for a particular message
     */
    void updateLogStatistics(const Oem7RawMessageIf::ConstPtr& raw_msg)
    {
      total_log_count_++;

      if(log_counts_.find(raw_msg->getMessageId()) == log_counts_.end())
      {
        log_counts_[raw_msg->getMessageId()] = 0;
      }
      log_counts_[raw_msg->getMessageId()]++;


      if((total_log_count_ % 10000) == 0)
      {
        outputLogStatistics();
      }
    }

    void publishOem7RawMsg(Oem7RawMessageIf::ConstPtr raw_msg)
    {
        novatel_oem7_msgs::Oem7RawMsg::Ptr oem7_raw_msg(new novatel_oem7_msgs::Oem7RawMsg);
        oem7_raw_msg->message_data.insert(
                                        oem7_raw_msg->message_data.end(),
                                        raw_msg->getMessageData(0),
                                        raw_msg->getMessageData(raw_msg->getMessageDataLength()));

        assert(oem7_raw_msg->message_data.size() == raw_msg->getMessageDataLength());

        oem7rawmsg_pub_.publish(oem7_raw_msg);
    } 


   /**
     * Called by ROS decoder with new raw messages
     */
    void onNewMessage(Oem7RawMessageIf::ConstPtr raw_msg)
    {
      NODELET_DEBUG_STREAM("onNewMsg: fmt= " << raw_msg->getMessageFormat()
                              <<   " type= " << raw_msg->getMessageType());

      // Discard all unknown messages; this is normally when dealing with responses to ASCII commands.
      if(raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_UNKNOWN)
      {
        ++unknown_msg_num_;
        NODELET_DEBUG_STREAM("Unknown:    [ID: "   << raw_msg->getMessageId()          <<
                                          "Type: " << raw_msg->getMessageType()        <<
                                          "Fmt: "  << raw_msg->getMessageFormat()      <<
                                          "Len: "  << raw_msg->getMessageDataLength()  <<
                                          "]");
        if(publish_unknown_oem7raw_)
        {
            publishOem7RawMsg(raw_msg);
        }
        else
        {
            ++discarded_msg_num_;
        }
      }
      else
      {
        if(raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_ASCII  ||
           raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_ABASCII)
        {
          extractGPGGAData(raw_msg);
          std::string msg(raw_msg->getMessageData(0), raw_msg->getMessageData(raw_msg->getMessageDataLength()));

          NODELET_DEBUG_STREAM(">---------------------------" << std::endl
                            << msg                            << std::endl
                            << "<---------------------------");
        }

        if(raw_msg->getMessageType() == Oem7RawMessageIf::OEM7MSGTYPE_RSP) // Response
        {
          std::string rsp(raw_msg->getMessageData(0), raw_msg->getMessageData(raw_msg->getMessageDataLength()));
          if(rsp.find_first_not_of(" /t/r/n") != std::string::npos && // ignore all-whitespace responses
             std::all_of(rsp.begin(), rsp.end(), [](char c){return std::isprint(c);})) // Ignore any corrupt responses
          {
            {
              std::lock_guard<std::mutex> lk(rsp_ready_mtx_);
              rsp_ = rsp;
            }
            rsp_ready_cond_.notify_one();
          }
          else
          {
            NODELET_ERROR_STREAM("Discarded corrupt ASCII response: '" << rsp << "'");
          }
        }
        else // Log
        {
          if( raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_BINARY  || 
              raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_SHORTBINARY  ||
             (raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_ASCII && isNMEAMessage(raw_msg)))
          {
            updateLogStatistics(raw_msg);

            msg_handler_->handleMessage(raw_msg);

            // Publish Oem7RawMsg if specified
            if(raw_msg_pub_.find(raw_msg->getMessageId()) != raw_msg_pub_.end())
            {
                publishOem7RawMsg(raw_msg);
            }

            if(publish_delay_sec_ > 0)
            {
              sleep(publish_delay_sec_);
            }
          }
        }
      }
    }


    /**
     * Service loop; drives Oem7 message decoder. onNewMessage is called on this thread.
     */
    void serviceLoopCb(const ros::TimerEvent& event)
    {
      msg_decoder->service();

      outputLogStatistics();

      NODELET_WARN("No more input from Decoder; Oem7MessageNodelet finished.");
    }


  public:
    Oem7MessageNodelet():
      recvr_loader_(          "bynav_ros_driver", "bynav_ros_driver::Oem7ReceiverIf"),
      oem7_msg_decoder_loader("bynav_ros_driver", "bynav_ros_driver::Oem7MessageDecoderIf"),
      total_log_count_(0),
      unknown_msg_num_(0),
      discarded_msg_num_(0),
      publish_delay_sec_(0),
      publish_unknown_oem7raw_(false),
      is_running_{false},
      auth_status_(NtripAuthStatus::kAuthFailed)
    {
    }

    ~Oem7MessageNodelet()
    {
      NODELET_DEBUG("~Oem7MessageNodelet");
    }
  };
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(bynav_ros_driver::Oem7MessageNodelet, nodelet::Nodelet)
