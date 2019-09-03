#include <boost/date_time.hpp>

using namespace boost::posix_time;

class Accelerator {
public:
	Accelerator(float ax, float ay, float az) : ax_(ax), ay_(ay), az_(az)
	{

	}
	float GetAx()
	{
		return ax_;
	}
	void SetAx(float value)
	{
		ax_ = value;
	}
private:
	float ax_, ay_, az_;
};

class IMU{
	Accelerator acc;
	float wx, wy, wz;
	float pitch, roll, yaw;
	float q0, q1, q2, q3;
	ptime UTC;
	time_duration sample_td;
	time_duration sync_td;
	float latitude, longtidue, altitude;
	float Ve, Vn, Vu;
};
