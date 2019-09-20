#ifndef YESENSE_IMU_DISPLAY_IMU_H_
#define YESENSE_IMU_DISPLAY_IMU_H_

#include <boost/date_time.hpp>

using namespace boost::posix_time;
using namespace boost::gregorian;

class Accelerator {
public:
	Accelerator(float ax, float ay, float az)
		: ax_(ax), ay_(ay), az_(az) { }
	float GetAx() { return ax_; }
	float GetAy() { return ay_; }
	float GetAz() { return az_; }
	void SetAx(float value) { ax_ = value; }
	void SetAy(float value) { ay_ = value; }
	void SetAz(float value) { az_ = value; }
private:
	float ax_, ay_, az_;
};

class AngularVelocity {
public:
	AngularVelocity(float wx, float wy, float wz)
		: wx_(wx), wy_(wy), wz_(wz) { }
	float GetWx() { return wx_; }
	float GetWy() { return wy_; }
	float GetWz() { return wz_; }
	void SetWx(float value) { wx_ = value; }
	void SetWy(float value) { wy_ = value; }
	void SetWz(float value) { wz_ = value; }
private:
	float wx_, wy_, wz_;
};

class EulerAngle {
public:
	EulerAngle(float pitch, float roll, float yaw)
		: pitch_(pitch), roll_(roll), yaw_(yaw) { }
	float GetPitch() { return pitch_; }
	float GetRoll()  { return roll_; }
	float GetYaw()   { return yaw_; }
	void SetPitcch(float value) { pitch_ = value; }
	void SetRoll(float value)   { roll_  = value; }
	void SetYaw(float value)    { yaw_   = value; }
private:
	float pitch_, roll_, yaw_;
};

class Quaternion {
public:
	Quaternion(float q0, float q1, float q2, float q3)
		: q0_(q0), q1_(q1), q2_(q2), q3_(q3) { }
	float GetQ0() { return q0_; }
	float GetQ1() { return q1_; }
	float GetQ2() { return q2_; }
	float GetQ3() { return q3_; }
	void SetQ0(float value) { q0_ = value; }
	void SetQ1(float value) { q1_ = value; }
	void SetQ2(float value) { q2_ = value; }
	void SetQ3(float value) { q3_ = value; }
private:
	float q0_, q1_, q2_, q3_;
};

class IMU{
public:
	IMU(int ax, int ay, int az, 
		int wx, int wy, int wz,
		int pitch, int roll, int yaw,
		std::string stime, long sample_td, long sync_td,
		int q0, int q1, int q2, int q3) :
	acc_(scale*ax,scale*ay,scale*az), aglv_(scale*wx, scale*wy, scale*wz),elag_(scale*pitch,scale*roll,scale*yaw),
	sample_td_(sample_td), sync_td_(sync_td),
	qt_(scale*q0,scale*q1,scale*q2,scale*q3)
	{
		UTC_ = time_from_string(stime);
	};
	Accelerator getAcc()
	{
		return acc_;
	}

private:
	float scale = 0.000001;
	Accelerator acc_;
	AngularVelocity aglv_;
	EulerAngle elag_;
	Quaternion qt_;
	ptime UTC_;
	long sample_td_;
	long sync_td_;
};

#endif
