#ifndef IMU_INTEGRATOR_UI_UTILITY_H
#define IMU_INTEGRATOR_UI_UTILITY_H

#include <thread>
#include <vector>
#include <iostream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/hex.hpp>

namespace tools {

	std::string AsciiToHex(std::string value)
	{
		std::stringstream ss;
		std::string results;
		for (int i = 0; i < value.size(); ++i)
		{
			ss << std::hex << (int)value[i];
		}
		results += ss.str();
		return results;
	}

	inline int HexToDec(unsigned char c1, unsigned char c2, unsigned char c3, unsigned char c4)
	{
		std::stringstream ss;
		ss << boost::format("%|02X|") % (int)(unsigned char)c4;
		ss << boost::format("%|02X|") % (int)(unsigned char)c3;
		ss << boost::format("%|02X|") % (int)(unsigned char)c2;
		ss << boost::format("%|02X|") % (int)(unsigned char)c1;
		std::string res = ss.str();
		unsigned int x;
		std::stringstream s1;
		s1 << std::hex << res;
		s1 >> x;
		return static_cast<int>(x);
	}
}
#endif