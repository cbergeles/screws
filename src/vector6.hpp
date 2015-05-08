//  Copyright (c) 2015  Christos Bergeles and Imperial College London

//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifndef VECTOR6_HPP
#define VECTOR6_HPP

#include "screwsInitLibrary.hpp"
#include "screwException.hpp"
#include "translation.hpp"

namespace screws
{
  /*!
   * \class Vector6
   * \ingroup libScrews
   * \brief Implements a 6x1 vector along with necessary operators.
   * \note Author: Christos Bergeles
   * \date 08th May 2015
   */
  template<class NumType>
  class SCREWS_EXPORT Vector6
  {
  public:

    template<class NumTypeTwist> friend class Twist;

    /// @brief Default constructor with zeros.
    explicit Vector6()
    {
    }

    /// @brief Default destructor.
    ~Vector6()
    {
    }

    /// @brief Vector based constructor.
    /// @param v0 the first 3x1 values.
    /// @param v1 the final 3x1 values.
    explicit Vector6(const Translation<NumType>& v0, const Translation<NumType>& v1)
    {
      _v0 = v0;
      _v1 = v1;
    }

    /// @brief Value based constructor.
    /// @param v0-v5, the elements of the vector.
    explicit Vector6(const NumType& v0, const NumType& v1, const NumType& v2,
                     const NumType& v3, const NumType& v4, const NumType& v5)
    {
      _v0.setData(v0, v1, v2);
      _v1.setData(v3, v4, v5);
    }

    // ** Additions **
    /// @brief Element-by-element addition.
    Vector6<NumType> operator +(const Vector6<NumType>& V) const
    {
      return Vector6(V._v0 + _v0, V._v1 + _v1);
    }
    /// @brief In-place element-by-element addition.
    const Vector6<NumType>& operator +=(const Vector6<NumType>& V)
    {
      _v0 = _v0 + V._v0;
      _v1 = _v1 + V._v1;

      return *this;
    }
    /// @brief Addition of a number to every element.
    Vector6 operator +(const NumType& value) const
    {
      return Vector6<NumType>(_v0 + value, _v1 + value);
    }
    /// @brief In-place addition of a number to every element.
    const Vector6& operator +=(const NumType& value)
    {
      _v0 += value;
      _v1 += value;

      return *this;
    }

    // ** Subtractions **
    /// @brief Element-by-element subtraction.
    Vector6<NumType> operator -(const Vector6<NumType>& V) const
    {
      return Vector6<NumType>(_v0 - V._v0, _v1 - V._v1);
    }
    /// @brief In-place element-by-element subtraction.
    const Vector6<NumType>& operator -=(const Vector6<NumType>& V)
    {
      _v0 = _v0 - V._v0;
      _v1 = _v1 - V._v1;

      return *this;
    }
    /// @brief Subtraction of a number from every element.
    Vector6<NumType> operator -(const NumType& value) const
    {
      return Vector6<NumType>(_v0 - value, _v1 - value);
    }
    /// @brief In-place subtraction of a number from every element.
    const Vector6<NumType>& operator -=(const NumType& value)
    {
      _v0 -= value;
      _v1 -= value;

      return *this;
    }

    // ** Multiplications **
    /// @brief Element-by-element multiplication.
    Vector6<NumType> operator *(const Vector6<NumType>& V) const
    {
      return Vector6(_v0 * V._v0, _v1 * V._v1);
    }
    /// @brief In-place Element-by-element multiplication.
    const Vector6<NumType>& operator *=(const Vector6<NumType>& V)
    {
      _v0 *= V._v0;
      _v1 *= V._v1;

      return *this;
    }
    /// @brief Multiplication of each element by a factor.
    Vector6<NumType> operator *(const NumType& scale) const
    {
      return Vector6<NumType>(_v0*scale, _v1*scale);
    }
    /// @brief In-place multiplication of each element by a scalar factor.
    const Vector6<NumType>& operator *=(const NumType& scale)
    {
      _v0 *= scale;
      _v1 *= scale;

      return *this;
    }

    // ** Divisions **
    /// @brief Element-by-element division.
    Vector6<NumType> operator /(const Vector6<NumType>& V) const
    {
      Translation<NumType> newV0;
      Translation<NumType> newV1;
      try
      {
        newV0 = _v0/V._v0;
        newV1 = _v1/V._v1;
      }
      catch(ScrewException e)
      {
        throw e;
      }

      return Vector6(newV0, newV1);
    }
    /// @brief Element-by-element division.
    const Vector6<NumType>& operator /=(const Vector6<NumType>& V)
    {
      try
      {
        _v0 /= V._v0;
        _v1 /= V._v1;
      }
      catch(ScrewException e)
      {
        throw e;
      }

      return *this;
    }
    /// @brief Division of each element by a factor.
    Vector6<NumType> operator /(const NumType& scale) const
    {
      if (scale == 0)
      {
        ScrewException e("Division by zero attempted.", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
      else
      {
        return Vector6(_v0 / scale, _v1 / scale);
      }
    }
    /// @brief Division of each element by a factor.
    const Vector6<NumType>& operator /=(const NumType& scale)
    {
      if (scale == 0)
      {
        ScrewException e("Division by zero attempted.", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
      else
      {
        _v0 = _v0 / scale;
        _v1 = _v1 / scale;

        return *this;
      }
    }

    // ** Accessors **
    /// @brief Accessor operator (write).
    /// @param the index in the vector.
    /// @return the index-the element of the vector.
    NumType& operator () (const unsigned int& index)
    {
      assert(index >= 0 && index < 6);
      if (index < 3)
      {
        return _v0(index);
      }
      else
      {
        return _v1(index - 3);
      }
    }
    /// @brief Accessor operator (read).
    const NumType& operator () (const unsigned int& index) const
    {
      assert(index >= 0 && index < 6);
      if (index < 3)
      {
        return _v0(index);
      }
      else
      {
        return _v1(index - 3);
      }
    }

    /// @brief Assignment operator.
    /// @return A copy of the right hand side argument.
    Vector6<NumType>& operator=(Vector6<NumType> prototype)
    {
      _v0 = prototype._v0;
      _v1 = prototype._v1;

      return *this;
    }

    /// @brief Equality operator.
    /// @return true if all element-by-element comparisons return true. Otherwise, false.
    bool operator ==(const Vector6<NumType>& V) const
    {
      return (_v0 == V._v0 && _v1 == V._v1);
    }
    /// @brief Inequality operator.
    /// @return true if one of the element-by-element comparisons return true. Otherwise, false.
    bool operator !=(const Vector6<NumType>& V) const
    {
      return (!(V == *this));
    }
    /// @brief Approximal equality operator, within a given epsilon or system precision.
    /// @param V compared vector.
    /// @param eps desired precision [default: machine precision].
    bool approxEq(const Vector6<NumType>& V, double eps = FLT_EPSILON) const
    {
      return (V._v0.approxEq(_v0, eps) && V._v1.approxEq(_v1, eps));
    }

    /// @brief Calculate the norm2 of the vector.
    /// @return the norm of the vector.
    NumType norm() const
    {
      return (NumType)sqrt(_v0(0)*_v0(0) + _v0(1)*_v0(1) + _v0(2)*_v0(2) +
                           _v1(0)*_v1(0) + _v1(1)*_v1(1) + _v1(2)*_v1(2));
    }

    /// @brief Normalise the vector.
    /// @return the normalised vector.
    Vector6<NumType> normalised() const
    {
      NumType n = norm();

      if (fabs(n) < 1e-10)
      {
        ScrewException s("Cannot normalise when norm is zero.", __FILE__, __FUNCTION__, __LINE__);
        throw s;
      }
      else
      {
        return Vector6<NumType>(_v0/n,_v1/n);
      }
    }

    /// @brief Find the dot product with another Vector6.
    /// @return the dot product with the given Vector6.
    NumType dot(const Vector6<NumType>& V)
    {
      return _v0.dot(V._v0) + _v1.dot(V._v1);
    }

  protected:

    Vector3<NumType> _v0;
    Vector3<NumType> _v1;

  };

  /// @brief Multiplication of each element by a scalar factor (commutativity).
  template <class NumType>
  Vector6<NumType> operator *(const NumType& scale, const Vector6<NumType>& V)
  {
    return V*scale;
  }

  /// @brief Print to stream.
  template <class NumType>
  std::ostream& operator<<(std::ostream& os, const Vector6<NumType>& V)
  {
    os << "[" << V(0) << ", " << V(1) << ", " << V(2) << ", "
       << V(3) << ", " << V(4) << ", " << V(5) << "]";

    return os;
  }

  // Convenience names
  using Vector6d = Vector6 < double >;
  using Vector6f = Vector6 < float >;

  template<typename NumType>
  using TwistCoordinates = Vector6 < NumType >;

  using TwistCoordinatesd = Vector6d;
  using TwistCoordinatesf = Vector6f;
};

#endif // VECTOR6_HPP
