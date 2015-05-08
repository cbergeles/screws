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

#ifndef TRANSLATION_HPP
#define TRANSLATION_HPP

#include <Eigen/Eigen>
#include <cfloat>

#include "screwsInitLibrary.hpp"
#include "screwException.hpp"

namespace screws
{
  /*!
   * \class Translation
   * \ingroup libScrews
   * \brief Implements a 3x1 translation along with necessary operators.
   * \note Author: Christos Bergeles
   * \date 25th April 2015
   */
  template<class NumType>
  class SCREWS_EXPORT Translation
  {
  public:

    template<class NumTypeRot> friend class Rotation;
    template<class NumTypeHomo> friend class HomogeneousTransform;
    template<class NumTypeVec> friend class Vector6;
    template<class NumTypeTw> friend class Twist;

    /// @brief Default constructor with zeros.
    explicit Translation()
    {
      setData((NumType)0, (NumType)0, (NumType)0);
    }

    /// @brief Default destructor.
    ~Translation()
    {

    }

    /// @brief Value based constructor.
    /// @param x the x-coordinate of the translation.
    /// @param y the y-coordinate of the translation.
    /// @param z the z-coordinate of the translation.
    explicit Translation(const NumType& x, const NumType& y, const NumType& z)
    {
      setData(x, y, z);
    }

    // ** Additions **
    /// @brief Element-by-element addition.
    Translation<NumType> operator +(const Translation<NumType>& T) const
    {
      return Translation<NumType>(T._data[0] + _data[0],
        T._data[1] + _data[1],
        T._data[2] + _data[2]);
    }
    /// @brief In-place element-by-element addition.
    const Translation<NumType>& operator +=(const Translation<NumType>& T)
    {
      _data = _data + T._data;
      return *this;
    }
    /// @brief Addition of a number to every element.
    Translation operator +(const NumType& value) const
    {
      return Translation<NumType>(_data[0] + value,
        _data[1] + value,
        _data[2] + value);
    }
    /// @brief In-place addition of a number to every element.
    const Translation& operator +=(const NumType& value)
    {
      _data[0] += value;
      _data[1] += value;
      _data[2] += value;

      return *this;
    }

    // ** Subtractions **
    /// @brief Element-by-element subtraction.
    Translation<NumType> operator -(const Translation<NumType>& T) const
    {
      return Translation<NumType>(_data[0] - T._data[0],
        _data[1] - T._data[1],
        _data[2] - T._data[2]);
    }
    /// @brief In-place element-by-element subtraction.
    const Translation<NumType>& operator -=(const Translation<NumType>& T)
    {
      _data = _data - T._data;
      return *this;
    }
    /// @brief Subtraction of a number from every element.
    Translation<NumType> operator -(const NumType& value) const
    {
      return Translation<NumType>(_data[0] - value,
        _data[1] - value,
        _data[2] - value);
    }
    /// @brief In-place subtraction of a number from every element.
    const Translation<NumType>& operator -=(const NumType& value)
    {
      _data[0] -= value;
      _data[1] -= value;
      _data[2] -= value;

      return *this;
    }

    // ** Multiplications **
    /// @brief Element-by-element multiplication.
    Translation<NumType> operator *(const Translation<NumType>& T) const
    {
      return Translation<NumType>(_data[0] * T._data[0],
        _data[1] * T._data[1],
        _data[2] * T._data[2]);
    }
    /// @brief In-place Element-by-element multiplication.
    const Translation<NumType>& operator *=(const Translation<NumType>& T)
    {
      _data[0] *= T._data[0];
      _data[1] *= T._data[1];
      _data[2] *= T._data[2];

      return *this;
    }
    /// @brief Multiplication of each element by a factor.
    Translation<NumType> operator *(const NumType& scale) const
    {
      return Translation<NumType>(_data[0] * scale,
        _data[1] * scale,
        _data[2] * scale);
    }
    /// @brief In-place multiplication of each element by a factor.
    const Translation<NumType>& operator *=(const NumType& scale)
    {
      _data[0] *= scale;
      _data[1] *= scale;
      _data[2] *= scale;

      return *this;
    }

    // ** Divisions **
    /// @brief Element-by-element division.
    /// @throw scews::ScrewException for division by zero.
    Translation<NumType> operator /(const Translation<NumType>& T) const
    {
      if (T._data[0] == 0 ||
        T._data[1] == 0 ||
        T._data[2] == 0)
      {
        ScrewException e("Division by zero attempted.", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
      else
      {
        return Translation<NumType>(_data[0] / T._data[0],
          _data[1] / T._data[1],
          _data[2] / T._data[2]);
      }
    }
    /// @brief Element-by-element division.
    /// @throw scews::ScrewException for division by zero.
    const Translation<NumType>& operator /=(const Translation<NumType>& T)
    {
      if (T._data[0] == 0 ||
        T._data[1] == 0 ||
        T._data[2] == 0)
      {
        ScrewException e("Division by zero attempted.", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
      else
      {
        _data[0] = _data[0] / T._data[0];
        _data[1] = _data[1] / T._data[1];
        _data[2] = _data[2] / T._data[2];

        return *this;
      }
    }
    /// @brief Division of each element by a factor.
    /// @throw scews::ScrewException for division by zero.
    Translation<NumType> operator /(const NumType& scale) const
    {
      if (scale == 0)
      {
        ScrewException e("Division by zero attempted.", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
      else
      {
        return Translation<NumType>(_data[0] / scale,
          _data[1] / scale,
          _data[2] / scale);
      }
    }
    /// @brief Division of each element by a factor.
    /// @throw scews::ScrewException for division by zero.
    const Translation<NumType>& operator /=(const NumType& scale)
    {
      if (scale == 0)
      {
        ScrewException e("Division by zero attempted.", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
      else
      {
        _data[0] = _data[0] / scale;
        _data[1] = _data[1] / scale;
        _data[2] = _data[2] / scale;

        return *this;
      }
    }

    // ** Accessors **
    /// @brief Accessor operator (write).
    /// @return the index-the element of the translation.
    NumType& operator () (const unsigned int& index)
    {
      assert(index >= 0 && index < 3);
      return _data[index];
    }
    /// @brief Accessor operator (read).
    const NumType& operator () (const unsigned int& index) const
    {
      assert(index >= 0 && index < 3);
      return _data[index];
    }

    /// @brief Assignment operator.
    /// @return A copy of the right hand side argument.
    Translation<NumType>& operator=(Translation<NumType> prototype)
    {
      _data = prototype._data;
      
      return *this;
    }
    
    /// @brief Equality operator.
    /// @return true if all element-by-element comparisons return true. Otherwise, false.
    bool operator ==(const Translation<NumType>& T) const
    {
      if (_data[0] != T._data[0] ||
        _data[1] != T._data[1] ||
        _data[2] != T._data[2])
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    /// @brief Inequality operator.
    /// @return true if one of the element-by-element comparisons return true. Otherwise, false.
    bool operator !=(const Translation<NumType>& T) const
    {
      if (_data[0] != T._data[0] ||
        _data[1] != T._data[1] ||
        _data[2] != T._data[2])
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    /// @brief Approximal equality operator, within a given epsilon or system precision.
    /// @param T compared translation.
    /// @param eps desired precision [default: machine precision].
    bool approxEq(const Translation<NumType>& T, double eps = FLT_EPSILON) const
    {
      if (fabs(T(0) - _data[0]) < eps &&
          fabs(T(1) - _data[1]) < eps &&
          fabs(T(2) - _data[2]) < eps)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /// @brief Calculate the norm2 of the translation.
    /// @return the norm of the translation.
    NumType norm() const
    {
      return (NumType)(_data.norm());
    }

    /// @brief Normalise the translation as if it were a vector.
    /// @return the normalised vector.
    /// @throw scews::ScrewException for zero translation.
    Translation<NumType> normalised() const
    {
      NumType n = _data.norm();

      if (fabs(n) < 1e-10)
      {
        ScrewException s("Cannot normalise when norm is zero.", __FILE__, __FUNCTION__, __LINE__);
        throw s;
      }
      else
      {
        return Translation<NumType>(_data[0]/n, _data[1]/n, _data[2]/n);
      }
    }

    /// @brief Find the cross product with a translation vector.
    /// @return the cross product with the given translation vector.
    Translation<NumType> cross(const Translation<NumType>& T)
    {
      return Translation<NumType>(
            _data[1]*T._data[2] - _data[2]*T._data[1],
            _data[2]*T._data[0] - _data[0]*T._data[2],
            _data[0]*T._data[1] - _data[1]*T._data[0]);
    }

    /// @brief Find the dot product with a translation vector.
    /// @return the dot product with the given translation vector.
    NumType dot(const Translation<NumType>& T)
    {
      return _data[0]*T._data[0] + _data[1]*T._data[1] + _data[2]*T._data[2];
    }

  protected:

    void setData(const NumType& x, const NumType& y, const NumType& z)
    {
      _data[0] = x;
      _data[1] = y;
      _data[2] = z;
    }

    Eigen::Matrix<NumType, 3, 1, 0, 3, 1> _data;

  };

  /// @brief Multiplication of each element by a scalar factor (commutativity).
  template <class NumType>
  Translation<NumType> operator *(const NumType& scale, const Translation<NumType>& T)
  {
    return T*scale;
  }

  /// @brief Print to stream.
  template <class NumType>
  std::ostream& operator<<(std::ostream& os, const Translation<NumType>& T)
  {
    os << "[" << T(0) << ", " << T(1) << ", " << T(2) << "]";

    return os;
  }

  // Convenience names
  using Translationd = Translation < double >;
  using Translationf = Translation < float >;

  template<typename NumType>
  using Vector3 = Translation < NumType >;

  using Vector3f = Translationf;
  using Vector3d = Translationd;
};

#endif // TRANSLATION_HPP
