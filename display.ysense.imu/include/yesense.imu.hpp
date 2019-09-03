
#ifndef IMU_INTEGRATOR_UI_YESENSE_IMU_HPP_
#define IMU_INTEGRATOR_UI_YESENSE_IMU_HPP_

#include <thread>
#include <vector>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/chrono.hpp>
#include <boost/format.hpp>
#include <boost/noncopyable.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/hex.hpp>

#include "glog/logging.h"

#include "buffered_async_serial.h"
#include "analysis_data.h"
#include "utility.h"

#include "imu.h"

using namespace tools;
using namespace vslam::model;

namespace drivers {

	class Yesense : boost::noncopyable
	{
		public:
			Yesense() = delete;
			Yesense(const std::string name, const int speed) :
				port_(name), baud_width_(speed)
			{	
				serial_.open(port_, baud_width_);
				boost::this_thread::sleep_for(boost::chrono::microseconds{ 100 });				
			}

			bool IsOpen()
			{
				return serial_.isOpen();				
			}

			std::vector<IMU> Pop()
			{
				std::vector<IMU> tmp;
				{
					boost::lock_guard<boost::mutex> lock(imu_cache_lock_);
					tmp.insert(tmp.begin(), imu_cache_.begin(), imu_cache_.end());
					imu_cache_.clear();
				}
				return tmp;
			}

			void AsyncStart() {
				boost::thread( boost::bind(&Yesense::ReadAndValidate, this) ).detach();
			}

			size_t Size()
			{
				size_t size;
				{
					boost::lock_guard<boost::mutex> lock(imu_cache_lock_);
					size = imu_cache_.size();					
				}
				return size;
			}

			void Stop() {
				stop_ = true;
			}

		private:
			//cost too much resource
			void ReadAndValidate()
			{
				try {
					while (1) {
						if (serial_.isOpen()) {
							std::string str = serial_.readStringUntil(start_bytes_);
							if (str.length() == received_bytes_) {
								str = start_bytes_hex_ + str;
								for (int i = 0; i < frame_length_; i++)
								{
									std::string out = boost::algorithm::unhex<std::string>({ str[2 * i], str[2 * i + 1] });
									g_protocol_data[i] = (int)out[0];
								}
								if (analysis_data(g_protocol_data, frame_length_) == analysis_ok) {
									IMU imu(HexToDec(
										g_protocol_data[7], g_protocol_data[8], g_protocol_data[9], g_protocol_data[10]),
										HexToDec(
											g_protocol_data[11], g_protocol_data[12], g_protocol_data[13], g_protocol_data[14]),
										HexToDec(
											g_protocol_data[15], g_protocol_data[16], g_protocol_data[17], g_protocol_data[18]),
										HexToDec(
											g_protocol_data[21], g_protocol_data[22], g_protocol_data[23], g_protocol_data[24]),
										HexToDec(
											g_protocol_data[25], g_protocol_data[26], g_protocol_data[27], g_protocol_data[28]),
										HexToDec(
											g_protocol_data[29], g_protocol_data[30], g_protocol_data[31], g_protocol_data[32]),
										HexToDec(
											g_protocol_data[63], g_protocol_data[64], g_protocol_data[65], g_protocol_data[66]),
										HexToDec(
											g_protocol_data[67], g_protocol_data[68], g_protocol_data[69], g_protocol_data[70]),
										HexToDec(
											g_protocol_data[71], g_protocol_data[72], g_protocol_data[73], g_protocol_data[74]),
										HexToDec(
											g_protocol_data[77], g_protocol_data[78], g_protocol_data[79], g_protocol_data[80]),
										HexToDec(
											g_protocol_data[81], g_protocol_data[82], g_protocol_data[83], g_protocol_data[84]),
										HexToDec(
											g_protocol_data[85], g_protocol_data[86], g_protocol_data[87], g_protocol_data[88]),
										HexToDec(
											g_protocol_data[89], g_protocol_data[90], g_protocol_data[91], g_protocol_data[92])
									);
									//printf("good data received, IMU data: a is: %f %f %f ", imu.ax, imu.ay, imu.az);
									//printf("angular velocity is: %f %f %f ", imu.wx, imu.wy, imu.wy);
									//printf("euler is: %f %f %f ", imu.r, imu.p, imu.y);
									//printf("queration is :%f %f %f %f\n", imu.qw, imu.qx, imu.qy, imu.qz);
									{
										boost::lock_guard<boost::mutex> lock(imu_cache_lock_);
										imu_cache_.push_back(imu);
									}
									
								}
								else {
									printf("receive error.\n");
									LOG(WARNING) << "frame validation error.";
								}
							}
						}
						if (stop_) {
							serial_.close();
							break;
						}
					}
				}
				catch (boost::system::system_error& e)
				{
					std::cout << "Error: " << e.what() << std::endl;
					return;
				}
				LOG(INFO) << "IMU read frame stoped.";
			}
			
			boost::mutex imu_cache_lock_;
			const std::string start_bytes_ = "YS";
			const std::string start_bytes_hex_ = "5953";
			const int received_bytes_ = 186;
			const int frame_length_ = 95;
			bool stop_ = false;
			BufferedAsyncSerial serial_;
			std::vector<IMU> imu_cache_;
			std::string port_;
			int baud_width_;
			unsigned char g_protocol_data[95] =
			{
					0x59 ,0x53 ,0x40 ,0x92 ,0x58 ,0x10 ,0x0c ,0x1b ,0xfd ,0xd3 ,0xff ,0x89 ,0x2c ,0xfe ,0xff , //14
					0x59 ,0x9c ,0x6a ,0xff ,0x20 ,0x0c ,0xd1 ,0xa2 ,0x02 ,0x00 ,0xfa ,0xb1 ,0x05 ,0x00 ,0x38 , //29
					0xd0 ,0x00 ,0x00 ,0x30 ,0x0c ,0x20 ,0xff ,0xf1 ,0x07 ,0xd0 ,0xa0 ,0xea ,0xf4 ,0xc0 ,0x8e , //44
					0xc6 ,0xef ,0x31 ,0x0c ,0xb4 ,0x08 ,0x02 ,0x00 ,0xa2 ,0x29 ,0xfd ,0xff ,0xb8 ,0xd8 ,0xfb , //59
					0xff ,0x40 ,0x0c ,0xcb ,0xe4 ,0xf4 ,0xff ,0xb1 ,0xbe ,0x09 ,0x00 ,0x30 ,0x32 ,0xb6 ,0xf6 , //74
					0x41 ,0x10 ,0xd4 ,0x31 ,0x03 ,0x00 ,0x4d ,0xec ,0xff ,0xff ,0x86 ,0xe5 ,0xff ,0xff ,0xa9 , //89
					0x14 ,0xf1 ,0xff ,0x12 ,0xf3
			};
	};
}

#endif //IMU_INTEGRATOR_UI_YESENSE_IMU_HPP_