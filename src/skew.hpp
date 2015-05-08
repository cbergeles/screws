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
#ifndef SKEW_H
#define SKEW_H

#include "screwsInitLibrary.hpp"
#include "screwException.hpp"
#include <Eigen/Eigen>
#include <cfloat>

namespace screws
{
  template<class NumType>
  class Rotation;
  template<class NumType>
  class Translation;
  
  /*!
   * \class Skew
   * \ingroup libScrews
   * \brief Implenets a class for the 3x3 skew symmetric rotation generators and their operations.
   * \note Author: Christos Bergeles
   * \date 5th May 2015
   */
  template<class NumType>
  class SCREWS_EXPORT Skew
  {
  public:
    template<class NumTypeRot> friend class Rotation;
    template<class NumTypeTrans> friend class Translation;
    template<class NumTypeTwist> friend class Twist;
    
    /// @brief Create a skew symmetric matrix out of a 3x1 vector.
    /// @param v the 3x1 vector, which also contains the angle information.
    explicit Skew(const Translation<NumType>& v)
    {
      _data(0, 0) = (NumType)0;
      _data(0, 1) = -v(2);
      _data(0, 2) = v(1);

      _data(1, 0) = v(2);
      _data(1, 1) = (NumType)0;
      _data(1, 2) = -v(0);

      _data(2, 0) = -v(1);
      _data(2, 1) = v(0);
      _data(2, 2) = (NumType)0;
    }
    
    /// @brief Create a skew symmetric matrix by taking the log of a rotation matrix.
    /// @param R the 3x3 rotation matrix.
    /// @param theta the extracted rotation angle.
    explicit Skew(const Rotation<NumType>& R)
    {
      // R will always be valid, otherwise it won't be a rotation.
      NumType theta = R.angle();
      if (theta == (NumType)0)
      {
        resetData();
      }
      else
      {
        _data = (R._data - R.inv()._data)/(2*sin(theta))*theta;
      }
    }
    
    /// @brief Default constructor with zeros. This correponds to the identity rotation.
    explicit Skew()
    {
      resetData();
    }
    
    /// Default destructor.
    ~Skew()
    {

    }
    
    /// @brief Transposes the skew.
    /// @return the transposed skew.
    Skew<NumType> transpose() const
    {
      Vector3<NumType> v = coordinates();

      return Skew<NumType>((NumType)(-1)*v);
    }
    
    /// @brief Calculate the exponential of the skew symmetric matrix. This corresponds to a rotation.
    /// @param theta the rotation magnitude.
    Rotation<NumType> exp(const NumType& theta = (NumType)1) const
    {
      if (theta == (NumType)0)
      {
        return Rotation<NumType>();
      }
      else
      {
        NumType mag = angle();

        if (fabs(mag) < FLT_EPSILON)
        {
          return Rotation<NumType>();
        }
        else
        {
          // Rodriguez formula
          Rotation<NumType> rGen;
          rGen._data = rGen._data +
              (_data/mag)*sin(mag*theta) + (_data*_data)/(mag*mag)*((NumType)1.0 - cos(mag*theta));

          return rGen;
        }
      }
    }
    
    /// @brief Normalise and remove magnitude from skew symmetric matrix. Only rotation axis information remains.
    /// @return the normalised skew.
    Skew<NumType> normalised() const
    {
      NumType mag = angle();

      if (mag > (NumType)0)
      {
        Skew<NumType> newSkew;
        newSkew._data = _data/mag;

        return newSkew;
      }
      else
      {
        return Skew<NumType>();
      }
    }

    /// @brief Calculate and return the norm of the skew.
    /// @return the norm of the skew.
    NumType norm() const
    {
      return coordinates().norm();
    }

    /// @brief Extract the rotation magnitude without normalising.
    /// @return the magnitude of the rotation.
    NumType angle() const
    {
      return (NumType)sqrt(_data(0, 1)*_data(0, 1) + _data(0, 2)*_data(0, 2) + _data(1, 2)*_data(1, 2));
    }

    /// @brief Extract the axis of rotation.
    /// @return the axis of rotation as a 3x1 vector.
    Vector3<NumType> axis() const
    {
      NumType val = angle();

      return Vector3<NumType>(_data(2, 1)/val, _data(0, 2)/val, _data(1, 0)/val);
    }

    /// @brief Return the requested element (read).
    /// @param i the index of the row.
    /// @param j the index of the column.
    /// @note No boundary check is performed.
    const NumType& operator () (const unsigned int& i, const unsigned int& j) const
    {
      assert(i >= 0 && i < 3 && j >= 0 && j < 3);
      return _data(i, j);
    }

    /// @brief Element-by-element addition of skew matrices.
    /// @note This operation corresponds to multiplication of the respective rotations.
    Skew<NumType> operator +(const Skew<NumType>& S) const
    {
      return Skew<NumType>(S.coordinates() + this->coordinates());
    }

    /// @brief Element-by-element in-place addition of skew matrices.
    /// @note This operation corresponds to multiplication of the respective rotations.
    const Skew<NumType>& operator +=(const Skew<NumType>& S)
    {
      Skew<NumType> newS(S.coordinates() + this->coordinates());

      _data = newS._data;

      return *this;
    }

    /// @brief Element-by-element multiplication with a value.
    /// @note This operation corresponds to scaling the rotation angle.
    Skew<NumType> operator *(const NumType& value) const
    {
      return Skew<NumType>(value*this->coordinates());
    }

    /// @brief Element-by-element in-place multiplication with a value.
    /// @note This operation corresponds to scaling the rotation angle.
    const Skew<NumType>& operator *=(const NumType& value)
    {
      Skew<NumType> newS(value*this->coordinates());

      _data = newS._data;

      return *this;
    }

    /// @brief Element-by-element exact equality operator.
    /// @return true if all elements are exactly equal
    bool operator ==(const Skew<NumType>& S) const
    {
      bool valid = true;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          valid = valid && (S(i, j) == _data(i, j));

      return valid;
    }

    /// @brief Inequality operator.
    /// @return true if one of the element-by-element comparisons return true. Otherwise, false.
    bool operator !=(const Skew<NumType>& S) const
    {
      if (S == *this)
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    /// @brief Approximal equality operator, within a given epsilon or system precision.
    /// @param S compared skew matrix.
    /// @param eps desired precision [default: machine precision].
    bool approxEq(const Skew<NumType>& S, double eps = FLT_EPSILON) const
    {
      if (fabs(S(0, 0) - _data(0, 0)) < eps &&
          fabs(S(1, 0) - _data(1, 0)) < eps &&
          fabs(S(2, 0) - _data(2, 0)) < eps &&
          fabs(S(0, 1) - _data(0, 1)) < eps &&
          fabs(S(1, 1) - _data(1, 1)) < eps &&
          fabs(S(2, 1) - _data(2, 1)) < eps &&
          fabs(S(0, 2) - _data(0, 2)) < eps &&
          fabs(S(1, 2) - _data(1, 2)) < eps &&
          fabs(S(2, 2) - _data(2, 2)) < eps)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /// @brief Generates a 3x1 vector given a skew symmetric matrix.
    /// @return the 3x1 vector.
    Vector3<NumType> coordinates() const
    {
      Vector3<NumType> v(-_data(1, 2), _data(0, 2), -_data(0, 1));

      return v;
    }
    
    /// @brief Perform verification that the matrix corresponds to a rotation.
    /// @return true if the matrix corresponds to a rotation, false otherwise.
    bool isValid() const
    {
      double val = 0;
      for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
          val += _data(i, j) + _data(j, i);

      if (val == 0)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
    
  protected:

    // Set all to zero.
    void resetData()
    {
      _data(0, 0) = _data(0, 1) = _data(0, 2) = (NumType)0;
      _data(1, 0) = _data(1, 1) = _data(1, 2) = (NumType)0;
      _data(2, 0) = _data(2, 1) = _data(2, 2) = (NumType)0;
    }

    // Data holder.
    Eigen::Matrix<NumType, 3, 3> _data;
  };

  /// @brief Print to stream.
  template <class NumType>
  std::ostream& operator<<(std::ostream& os, const Skew<NumType>& S)
  {
    os << "[" << S(0, 0) << ", " << S(0, 1) << ", " << S(0, 2) << ";\n";
    os << " " << S(1, 0) << ", " << S(1, 1) << ", " << S(1, 2) << ";\n";
    os << " " << S(2, 0) << ", " << S(2, 1) << ", " << S(2, 2) << "]";

    return os;
  }

  /// @brief Element-by-element pre-multiplication with a value.
  /// @note This operation corresponds to scaling the rotation angle.
  template <class NumType>
  Skew<NumType> operator *(const NumType& value, const Skew<NumType>& S)
  {
      return S*value;
  }

  // Convenience names
  using Skewd = Skew < double >;
  using Skewf = Skew < float >;
};

#endif // SKEW_H
