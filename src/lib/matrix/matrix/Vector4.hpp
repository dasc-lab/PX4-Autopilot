/**
 * @file Vector4.hpp
 *
 * 4D vector class.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */

#pragma once

#include "math.hpp"

namespace matrix
{

template <typename Type, size_t M, size_t N>
class Matrix;

template <typename Type, size_t M>
class Vector;

template <typename Type>
class Dcm;

template<typename Type>
class Vector4 : public Vector<Type, 4>
{
public:

	using Matrix41 = Matrix<Type, 4, 1>;

	Vector4() = default;

	Vector4(const Matrix41 &other) :
		Vector<Type, 4>(other)
	{
	}

	explicit Vector4(const Type data_[4]) :
		Vector<Type, 4>(data_)
	{
	}

	Vector4(Type x, Type y, Type z, Type w)
	{
		Vector4 &v(*this);
		v(0) = x;
		v(1) = y;
		v(2) = z;
    v(3) = w;
	}

	template<size_t P, size_t Q>
	Vector4(const Slice<Type, 4, 1, P, Q> &slice_in) : Vector<Type, 4>(slice_in)
	{
	}

	template<size_t P, size_t Q>
	Vector4(const Slice<Type, 1, 4, P, Q> &slice_in) : Vector<Type, 4>(slice_in)
	{
	}

	/**
	 * Override matrix ops so Vector4 type is returned
	 */

	inline Vector4 operator+(Vector4 other) const
	{
		return Matrix41::operator+(other);
	}

	inline Vector4 operator+(Type scalar) const
	{
		return Matrix41::operator+(scalar);
	}

	inline Vector4 operator-(Vector4 other) const
	{
		return Matrix41::operator-(other);
	}

	inline Vector4 operator-(Type scalar) const
	{
		return Matrix41::operator-(scalar);
	}

	inline Vector4 operator-() const
	{
		return Matrix41::operator-();
	}

	inline Vector4 operator*(Type scalar) const
	{
		return Matrix41::operator*(scalar);
	}

	inline Type operator*(Vector4 b) const
	{
		return Vector<Type, 4>::operator*(b);
	}

	inline Vector4 operator%(const Matrix41 &b) const
	{
		return (*this).cross(b);
	}

	/**
	 * Override vector ops so Vector4 type is returned
	 */
	inline Vector4 unit() const
	{
		return Vector4(Vector<Type, 4>::unit());
	}

	inline Vector4 normalized() const
	{
		return unit();
	}

	const Slice<Type, 2, 1, 4, 1> xy() const
	{
		return Slice<Type, 2, 1, 4, 1>(0, 0, this);
	}

	Slice<Type, 2, 1, 4, 1> xy()
	{
		return Slice<Type, 2, 1, 4, 1>(0, 0, this);
	}


};

using Vector4f = Vector4<float>;
using Vector4d = Vector4<double>;

} // namespace matrix
