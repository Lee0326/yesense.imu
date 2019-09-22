
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
#include "utility.h"

#include "imu.h"

using namespace tools;

namespace drivers {

	class Yesense : boost::noncopyable
	{
		public:
			// Yesense() = delete;
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
			std::string time = "2019-9-17 17:32:00";
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
								if (true) {
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
										time,
										HexToDec(
											g_protocol_data[95], g_protocol_data[96], g_protocol_data[97], g_protocol_data[98]),
										HexToDec(
											g_protocol_data[101], g_protocol_data[102], g_protocol_data[103], g_protocol_data[104]),
										HexToDec(
											g_protocol_data[77], g_protocol_data[78], g_protocol_data[79], g_protocol_data[80]),
										HexToDec(
											g_protocol_data[81], g_protocol_data[82], g_protocol_data[83], g_protocol_data[84]),
										HexToDec(
											g_protocol_data[85], g_protocol_data[86], g_protocol_data[87], g_protocol_data[88]),
										HexToDec(
											g_protocol_data[89], g_protocol_data[90], g_protocol_data[91], g_protocol_data[92])
									);
									printf("good data received, IMU data: a is: %f %f %f ", imu.getAcc().GetAx(), imu.getAcc().GetAy(), imu.getAcc().GetAz());
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
			const int received_bytes_ = 210;
			const int frame_length_ = 107;
			bool stop_ = false;
			BufferedAsyncSerial serial_;
			std::vector<IMU> imu_cache_;
			std::string port_;
			int baud_width_;
			unsigned char g_protocol_data[107] =
			{
				  0x59 ,0x53 ,0x01 ,0x00 ,0x64 ,0x10 ,0x0C ,0x04 ,0xAD ,0xFE ,0xFF ,0x2C ,0x31 ,0x05 ,0x00 , //14
				  0xCA ,0x8C ,0x93 ,0x00 ,0x20 ,0x0C ,0x32 ,0xA0 ,0xFD ,0xFF ,0x63 ,0xAC ,0x04 ,0x00 ,0xA6 , //29
				  0x59 ,0xFF ,0xFF ,0x30 ,0x0C ,0x20 ,0x15 ,0x8A ,0xF5 ,0x40 ,0xE9 ,0x43 ,0x0B ,0xE0 ,0x2C , //44
				  0xDD ,0xF7 ,0x31 ,0x0C ,0x74 ,0x52 ,0xFD ,0xFF ,0x48 ,0xE2 ,0x02 ,0x00 ,0xCC ,0xEA ,0xFD , //59
				  0xFF ,0x40 ,0x0C ,0x79 ,0x1F ,0x07 ,0x00 ,0xDB ,0x7B ,0x1F ,0x00 ,0xBA ,0xFE ,0xFF ,0xFF , //74
				  0x41 ,0x10 ,0x72 ,0x41 ,0x0F ,0x00 ,0x54 ,0x46 ,0x00 ,0x00 ,0xE8 ,0x0F ,0x00 ,0x00 ,0xB4 , //89
				  0xFF ,0xFF ,0xFF ,0x51 ,0x04 ,0xDA ,0x1A ,0x04 ,0x00 ,0x52 ,0x04 ,0xD7 ,0x25 ,0x04 ,0x00 , //104
				  0xB6 ,0x5E,
			};
	};
}

#endif //IMU_INTEGRATOR_UI_YESENSE_IMU_HPP_