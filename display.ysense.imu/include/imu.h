#include <boost/date_time.hpp>

using namespace boost::posix_time;

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

class Position {
public:
	Position(float latitude, float longitude, float altitude)
		: latitude_(latitude), longitude_(longitude), altitude_(altitude) { }
	float GetLatitude()  { return latitude_; }
	float GetLongitude() { return longitude_; }
	float GetAltitude()  { return altitude_; }
	void SetLatitude(float value)  { latitude_ = value; }
	void SetLongitude(float value) { longitude_ = value; }
	void SetAltitude(float value)  { altitude_ = value; }
private:
	float latitude_, longitude_, altitude_;
};

class Velocity {
public:
	Velocity(float Ve, float Vn, float Vu)
		: Ve_(Ve), Vn_(Vn), Vu_(Vu) { }
	float GetVe() { return Ve_; }
	float GetVn() { return Vn_; }
	float GetVu() { return Vu_; }
	void SetVe(float value) { Ve_ = value; }
	void SetVn(float value) { Vn_ = value; }
	void SetVu(float value) { Vu_ = value; }
private:
	float Ve_, Vn_, Vu_;
};


class IMU{
	Accelerator acc;
	AngularVelocity aglv; 
	EulerAngle elag;
	Quaternion qt;
	ptime UTC;
	time_duration sample_td;
	time_duration sync_td;
	Position pos;
	Velocity vel;
};
