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

#ifndef ROTATION_H
#define ROTATION_H

#include "screwsInitLibrary.hpp"
#include "screwException.hpp"
#include <Eigen/Eigen>
#include <cfloat>

namespace screws
{
  template<class NumType>
  class Translation;
  template<class NumType>
  class Skew;

  /*!
  * \class Rotation
  * \ingroup libScrews
  * \brief Implements a 3x3 rotation along with basic operators.
  * \note Author: Christos Bergeles
  * \date 27th April 2015
  */
  template<class NumType>
  class SCREWS_EXPORT Rotation
  {
  public:
    template<class NumTypeTrans> friend class Translation;
    template<class NumTypeSkew> friend class Skew;
    template<class NumTypeTwist> friend class Twist;

    /// @brief Construct a 3x3 identity rotation matrix.
    explicit Rotation()
    {
      resetData();
    }

    /// @brief Construct the 3x3 matrix from vectors.
    /// @param c0 the vector that will become the first column of the rotation matrix.
    /// @param c1 the vector that will become the second column of the rotation matrix.
    /// @param c2 the vector that will become the third column of the rotation matrix.
    /// @note The constructor checks if the resulting matrix has orthonormal columns.
    explicit Rotation(const Translation<NumType>& c0,
      const Translation<NumType>& c1,
      const Translation<NumType>& c2)
    {
      setData(c0(0), c1(0), c2(0), c0(1), c1(1), c2(1), c0(2), c1(2), c2(2));
    }

    /// @brief Construct a 3x3 rotation given common axes.
    /// @param axis "x", "y", or "z" for common rotations.
    /// @param theta the rotation angle.
    /// @throw screws::ScrewException for wrong axis identifier.
    /// @throw scews::ScrewException for out of bounds angle.
    explicit Rotation(const char& axis, const NumType& theta)
    {
      resetData();
      switch (axis)
      {
        case 'x':
        {
          construct(1, 0, 0, theta);
          break;
        }
        case 'y':
        {
          construct(0, 1, 0, theta);
          break;
        }
        case 'z':
        {
          construct(0, 0, 1, theta);
          break;
        }
        default:
        {
          ScrewException e("Only x, y, z are supported as axes.",
                           __FILE__,
                           __FUNCTION__,
                           __LINE__);
          throw e;
          break;
        }
      }
    }

    /// @brief Construct a 3x3 rotation given axis and angle.
    /// @param axisVector the axis of the rotation.
    /// @param theta the rotation angle.
    /// @throw scews::ScrewException for out of bounds angle.
    /// @note If axisVector is not normal, it is normalised.
    explicit Rotation(const Vector3<NumType>& axisVector, const NumType& theta)
    {
      Vector3<NumType> normalisedAxisVector = axisVector.normalised();
      construct(normalisedAxisVector(0), normalisedAxisVector(1), normalisedAxisVector(2), theta);
    }

    /// @brief Create a rotation instance given a Translation that acts as the z axis.
    /// @note  x and y are arbitrarily defined to generate a right-handed system.
    /// @param v the 3x1 orientation vector.
    explicit Rotation(const Vector3<NumType>& zAxis)
    {
      // z axis
      screws::Vector3<NumType> z = zAxis.normalised();

      // random x axis
      screws::Vector3<NumType> x((NumType)(0.3*rand())/RAND_MAX,
                                 (NumType)(0.5*rand())/RAND_MAX,
                                 (NumType)(0.4*rand())/RAND_MAX);
      x = x.normalised();

      // perpendicular y axis
      screws::Vector3<NumType> y = z.cross(x);
      y = y.normalised();

      // perpendicular x axis
      x = y.cross(z);

      setData(x(0), y(0), z(0),
        x(1), y(1), z(1),
        x(2), y(2), z(2));
    }

    /// Default destructor.
    ~Rotation()
    {

    }

    /// @brief Return the axis of rotation.
    /// @return the axis of rotation as a 3x1 vector.
    Vector3<NumType> axis() const
    {
      Vector3<NumType> axisVec;
      NumType angleVal;

      calculateAxisAndAngle(axisVec, angleVal);

      return axisVec;
    }

    /// @brief Return the magnitude of the rotation. It calculates the axis internally.
    /// @return the magnitude of rotation around the axis.
    NumType angle() const
    {
      NumType angleVal;
      Vector3<NumType> axisVec;

      calculateAxisAndAngle(axisVec, angleVal);

      return angleVal;
    }

    /// @brief Return the roll-pitch-yaw angles.
    /// @return a 3x1 vector of the rpy angles.
    /// @note Angles are returned in the [0, 2\pi] domain.
    Vector3<NumType> rpy() const
    {
      Vector3<NumType> v;
      NumType roll, pitch, yaw;
      NumType alpha, beta, gamma;

      beta = atan2(-_data(2, 0), sqrt(_data(0, 0)*_data(0, 0) + _data(1, 0)*_data(1, 0)));
      if (fabs(beta - M_PI_2) < FLT_EPSILON)
      {
        alpha = 0;
        gamma = atan2(_data(0, 1), _data(1, 1));
      }
      else if (fabs(beta + M_PI_2) < FLT_EPSILON)
      {
        alpha = 0;
        gamma = -atan2(_data(0, 1), _data(1, 1));
      }
      else
      {
        alpha = atan2(_data(1, 0)/cos(beta), _data(0, 0)/cos(beta));
        gamma = atan2(_data(2, 1)/cos(beta), _data(2, 2)/cos(beta));
      }

      roll = gamma;
      pitch = beta;
      yaw = alpha;

      if (fabs(roll - M_PI) < FLT_EPSILON && fabs(yaw - M_PI) < FLT_EPSILON)
      {
        roll = 0;
        yaw = 0;
        pitch = M_PI - pitch;
      }

      if (roll < 0)
      {
        roll += 2*M_PI;
      }
      if (pitch < 0)
      {
        pitch += 2*M_PI;
      }

      if (yaw < 0)
      {
        yaw += 2*M_PI;
      }

      v(0) = roll;
      v(1) = pitch;
      v(2) = yaw;

      return v;
    }

    /// @brief Invert by taking the transpose.
    /// @return the inverted rotation matrix.
    Rotation<NumType> inv() const
    {
      Eigen::Matrix<NumType, 3, 3, 0, 3, 3> newData = _data.transpose();

      Rotation<NumType> RtoReturn;
      
      RtoReturn.setData(newData(0, 0), newData(0, 1), newData(0, 2),
        newData(1, 0), newData(1, 1), newData(1, 2),
        newData(2, 0), newData(2, 1), newData(2, 2));
      
      return RtoReturn;
    }

    /// @brief Return the skew (log) symmetric matrix corresponding to this rotation.
    /// @return: the skew symmetric matrix.
    Skew<NumType> log() const
    {
      return Skew<NumType>(*this);
    }

    /// @brief Return the skew (log) symmetric matrix corresponding to this rotation
    /// @return: the skew symmetric matrix.
    Skew<NumType> skew() const
    {
      return log();
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

    /// @brief Multiplication operator.
    Rotation<NumType> operator *(const Rotation<NumType>& R) const
    {
      Eigen::Matrix<NumType, 3, 3, 0, 3, 3> newData = _data*R._data;

      Rotation<NumType> RtoReturn;
      RtoReturn.setData(newData(0, 0), newData(0, 1), newData(0, 2),
        newData(1, 0), newData(1, 1), newData(1, 2),
        newData(2, 0), newData(2, 1), newData(2, 2));
      
      return RtoReturn;
    }
    /// @brief In-place multiplication operator.
    const Rotation<NumType>& operator *=(const Rotation<NumType>& R)
    {
      _data = _data*R._data;

      return *this;
    }
    
    /// @brief Equality operator.
    /// @return true if all element-by-element comparisons return true. Otherwise, false.
    bool operator==(const Rotation<NumType>& R) const
    {
      bool valid = true;
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
          valid = valid && (R(i, j) == _data(i, j));

      return valid;
    }
    
    /// @brief Inequality operator.
    /// @return true if one of the element-by-element comparisons return true. Otherwise, false.
    bool operator !=(const Rotation<NumType>& R) const
    {
      if (R == *this)
      {
        return false;
      }
      else
      {
        return true;
      }
    }
    
    /// @brief Approximal equality operator, within a given epsilon or system precision.
    /// @param R compared rotation.
    /// @param eps desired precision [default: machine precision].
    bool approxEq(const Rotation<NumType>& R, double eps = FLT_EPSILON) const
    {
      if (fabs(R(0, 0) - _data(0, 0)) < eps &&
          fabs(R(1, 0) - _data(1, 0)) < eps &&
          fabs(R(2, 0) - _data(2, 0)) < eps &&
          fabs(R(0, 1) - _data(0, 1)) < eps &&
          fabs(R(1, 1) - _data(1, 1)) < eps &&
          fabs(R(2, 1) - _data(2, 1)) < eps &&
          fabs(R(0, 2) - _data(0, 2)) < eps &&
          fabs(R(1, 2) - _data(1, 2)) < eps &&
          fabs(R(2, 2) - _data(2, 2)) < eps)
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /// @brief Multiplication with a 3x1 Vector acting as a point.
    Vector3<NumType> operator *(const Vector3<NumType>& T) const
    {
      Eigen::Matrix<NumType, 3, 1, 0, 3, 1> newData = _data*T._data;

      return Translation<NumType>(newData(0), newData(1), newData(2));
    }

    /// @brief Perform verification that matrix is indeed in approriate format,
    /// by checking if the determinant is zero, and if the columns are othogonal
    /// to each other and of magnitude one.
    /// @return true if the rotation is valid, false otherwise.
    /// @throw scews::ScrewException for invalid rotation.
    bool isValid() const
    {
      double determinant = _data.determinant();
      double col0norm = _data(0, 0) * _data(0, 0) + _data(1, 0) * _data(1, 0) + _data(2, 0) * _data(2, 0);
      double col1norm = _data(0, 1) * _data(0, 1) + _data(1, 1) * _data(1, 1) + _data(2, 1) * _data(2, 1);
      double col2norm = _data(0, 2) * _data(0, 2) + _data(1, 2) * _data(1, 2) + _data(2, 2) * _data(2, 2);

      double col01norm = _data(0, 0) * _data(0, 1) + _data(1, 0) * _data(1, 1) + _data(2, 0) * _data(2, 1);
      double col02norm = _data(0, 0) * _data(0, 2) + _data(1, 0) * _data(1, 2) + _data(2, 0) * _data(2, 2);
      double col12norm = _data(0, 1) * _data(0, 2) + _data(1, 1) * _data(1, 2) + _data(2, 1) * _data(2, 2);

      bool validity = (fabs(determinant - 1.0) < FLT_EPSILON &&
        fabs(col0norm - 1) < FLT_EPSILON && fabs(col1norm - 1) < FLT_EPSILON && fabs(col2norm - 1) < FLT_EPSILON &&
        fabs(col01norm) < FLT_EPSILON && fabs(col02norm) < FLT_EPSILON && fabs(col12norm) < FLT_EPSILON);

      if (!validity)
      {
        char s[200];
        sprintf(s, "Determinant: %f,\n"
                   "col0norm = %f, col1norm = %f, col2norm = %f,\n"
                   "col01norm = %f, col02norm = %f, col12norm = %f",
                   determinant, col0norm, col1norm, col2norm,
                   col01norm, col02norm, col12norm);
        throw ScrewException(s, __FILE__, __FUNCTION__, __LINE__);
      }
      return true;
    }

  protected:

    // Basic setter of data for the general axis-angle case.
    void construct(const NumType& ux,
      const NumType& uy,
      const NumType& uz,
      const NumType& theta)
    {
      if (theta < 0 || theta >= 2*M_PI)
      {
        ScrewException s("Only angles within [0, 2pi) are supported", __FILE__,
                         __FUNCTION__, __LINE__);
        throw s;
      }

      NumType cs = cos(theta);
      NumType ss = sin(theta);

      _data(0, 0) = cs + ux*ux*(1 - cs);
      _data(0, 1) = ux*uy*(1 - cs) - uz*ss;
      _data(0, 2) = ux*uz*(1 - cs) + uy*ss;

      _data(1, 0) = uy*ux*(1 - cs) + uz*ss;
      _data(1, 1) = cs + uy*uy*(1 - cs);
      _data(1, 2) = uy*uz*(1 - cs) - ux*ss;

      _data(2, 0) = ux*uz*(1 - cs) - uy*ss;
      _data(2, 1) = uz*uy*(1 - cs) + ux*ss;
      _data(2, 2) = cs + uz*uz*(1 - cs);
    }

    // Helper to set the data.
    void setData(const NumType& r00, const NumType& r01, const NumType& r02,
      const NumType& r10, const NumType& r11, const NumType& r12,
      const NumType& r20, const NumType& r21, const NumType& r22)
    {
      _data(0, 0) = r00; _data(0, 1) = r01; _data(0, 2) = r02;
      _data(1, 0) = r10; _data(1, 1) = r11; _data(1, 2) = r12;
      _data(2, 0) = r20; _data(2, 1) = r21; _data(2, 2) = r22;

      if (!isValid())
      {
        std::cout << *this;
        resetData();
        ScrewException e("Rotation matrix not orthonormal", __FILE__, __FUNCTION__, __LINE__);
        throw e;
      }
    }

    // Resets the data to the unit matrix.
    void resetData()
    {
      _data(0, 0) = _data(1, 1) = _data(2, 2) = (NumType)1;
      _data(0, 1) = _data(0, 2) = (NumType)0;
      _data(1, 0) = _data(1, 2) = (NumType)0;
      _data(2, 0) = _data(2, 1) = (NumType)0;
    }
    
    // Calculates the axis and the angle in one go.
    void calculateAxisAndAngle(Vector3<NumType>& axisVec, NumType& angleVal) const
    {
      NumType tr = (NumType)(0.5)*(_data(0, 0) + _data(1, 1) + _data(2, 2) - 1);

      // Only in case of numerical errors
      if (tr < -1)
      {
        tr = (NumType)(-1);
      }
      else if (tr > 1)
      {
        tr = (NumType)(1);
      }

      angleVal = (NumType)acos(tr);
      
      axisVec(0) = _data(2, 1) - _data(1, 2);
      axisVec(1) = _data(0, 2) - _data(2, 0);
      axisVec(2) = _data(1, 0) - _data(0, 1);

      NumType normV = axisVec.norm();
      if (fabs(normV) > 1e-10)
      {
        axisVec /= normV;
      }
      else
      {
        axisVec(0) = 0;
        axisVec(1) = 0;
        axisVec(2) = 1;
      }

      if (axisVec(0) < 0)
      {
        axisVec = -1.0*axisVec;
        angleVal = (NumType)(2 * M_PI) - angleVal;
      }
    }

    // Data holder.
    Eigen::Matrix<NumType, 3, 3> _data;
  };

  /// @brief Print to stream.
  template <class NumType>
  std::ostream& operator<<(std::ostream& os, const Rotation<NumType>& R)
  {
    os << "[" << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ";\n";
    os << " " << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ";\n";
    os << " " << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2) << "]";

    return os;
  }

  // Convenience names
  using Rotationd = Rotation < double >;
  using Rotationf = Rotation < float >;
};

#endif // ROTATION_H
