#ifndef __VECTOR__
#define __VECTOR__

#include <memory>
#include <iostream>

class Vector3
{
public:
	Vector3(float, float, float);
	Vector3(const Vector3 &);

	Vector3();

	float distanceTo(const Vector3&) const;
	float squaredDistanceTo(const Vector3&) const;

	float magnitude() const;

	void normalize();

	static float angle(const Vector3&, const Vector3&, const Vector3&);

	static float dot(const Vector3&, const Vector3&);

	static Vector3 cross(Vector3, const Vector3&);

	static Vector3 zup();

	float x, y, z;

	// OPERATORS

	Vector3 &operator/= (const float& b);
	Vector3 &operator+= (const Vector3& b);
	Vector3 &operator+= (const float& b);
	Vector3 &operator-= (const Vector3& b);
	Vector3 &operator-= (const float& b);
	Vector3 &operator*= (float b);
	float operator[] (const size_t &);
	bool operator==(const Vector3&) const;

	friend Vector3 operator+ (Vector3 a, float b);
	friend Vector3 operator- (Vector3 a, float b);
	friend Vector3 operator/ (Vector3 a, float b);
	friend Vector3 operator* (Vector3 a, float b);
	friend Vector3 operator* (float b, Vector3 a);
	friend Vector3 operator- (Vector3 a, const Vector3& b);
	friend Vector3 operator+ (Vector3 a, const Vector3& b);

	friend bool operator>=(const Vector3&, const Vector3&);
	friend bool operator<=(const Vector3&, const Vector3&);

    friend std::ostream& operator<< (std::ostream& a, const Vector3& b);
};

typedef std::shared_ptr<Vector3> sVector3;

#endif
