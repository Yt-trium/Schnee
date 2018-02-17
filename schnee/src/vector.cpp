#include "vector.h"

#include <cmath>

Vector3::Vector3(float px, float py, float pz) :
	x(px), y(py), z(pz)
{

}

Vector3::Vector3(const Vector3 & other) : Vector3(other.x, other.y, other.z)
{

}

Vector3::Vector3() : Vector3(0, 0, 0) { } ;

float Vector3::distanceTo(const Vector3 & o) const
{
	return sqrt(pow(o.x - x, 2) + pow(o.y - y, 2) + pow(o.z - z, 2));
}

float Vector3::squaredDistanceTo(const Vector3 & o) const
{
	return pow(o.x - x, 2) + pow(o.y - y, 2) + pow(o.z - z, 2);
}

float Vector3::magnitude() const
{
	return sqrt(x * x + y * y + z * z);
}

void Vector3::normalize()
{
	float mag = magnitude();
	x /= mag;
	y /= mag;
	z /= mag;
}

float Vector3::angle(const Vector3 & va, const Vector3 & vb, const Vector3 & vc)
{
	Vector3 v1 = va - vb;
	Vector3 v2 = vc - vb;

	v1 /= v1.magnitude();
	v2 /= v2.magnitude();

	return acos(Vector3::dot(v1, v2));
}

float Vector3::triangleArea(const Vector3 & va, const Vector3 & vb, const Vector3 & vc)
{
	// Edge lengths
	float a = va.distanceTo(vb),
		  b = vb.distanceTo(vc),
		  c = vb.distanceTo(va);

	// semiperimiter
	float s = (a + b + c) * 0.5f;

	// area
	return sqrt(s * (s - a) * (s - b) * (s - c));
}

float Vector3::dot(const Vector3 & va, const Vector3 & vb)
{
	return va.x * vb.x + va.y * vb.y + va.z * vb.z;
}

Vector3 Vector3::cross(Vector3 res, const Vector3 & b)
{
	Vector3 a = res;
	res.x = a.y * b.z - a.z * b.y;
	res.y = a.z * b.x - a.x * b.z;
	res.z = a.x * b.y - a.y * b.x;
	return res;
}

Vector3 Vector3::zup()
{
	return Vector3(0, 0, 1);
}

Vector3 &Vector3::operator/=(const float &b)
{
	x /= b;
	y /= b;
	z /= b;
	return *this;
}

Vector3 &Vector3::operator+=(const Vector3 &b)
{
	x += b.x;
	y += b.y;
	z += b.z;
	return *this;
}

Vector3 &Vector3::operator+=(const float &b)
{
	x += b;
	y += b;
	z += b;
	return *this;
}

Vector3 &Vector3::operator-=(const Vector3 &b)
{
	x -= b.x;
	y -= b.y;
	z -= b.z;
	return *this;
}

Vector3 &Vector3::operator-=(const float &b)
{
	x -= b;
	y -= b;
	z -= b;
	return *this;
}

Vector3 &Vector3::operator*=(float b)
{
	x *= b;
	y *= b;
	z *= b;
	return *this;
}

bool Vector3::operator==(const Vector3 & a) const
{
	return a.x == x && a.y == y && a.z == z;
}

Vector3 operator+(Vector3 a, const Vector3 &b)
{
	a += b;
	return a;
}

Vector3 operator-(Vector3 a, const Vector3 &b)
{
	a -= b;
	return a;
}

Vector3 operator+(Vector3 a, float b)
{
	a += b;
	return a;
}

Vector3 operator-(Vector3 a, float b)
{
	a -= b;
	return a;
}

Vector3 operator/(Vector3 a, float b)
{
	a /= b;
	return a;
}

Vector3 operator*(Vector3 a, float b)
{
	a *= b;
	return a;
}

Vector3 operator*(float b, Vector3 a)
{
	a *= b;
	return a;
}

bool operator>=(const Vector3 & a, const Vector3 & b)
{
	return a.x >= b.x && a.y >= b.y && a.z >= b.z;
}

bool operator<=(const Vector3 & a, const Vector3 & b)
{
	return a.x <= b.x && a.y <= b.y && a.z <= b.z;
}


std::ostream &operator<<(std::ostream &a, const Vector3 &b)
{
	a << "(" << b.x << ", " << b.y << ", " << b.z << ")";
	return a;
}

