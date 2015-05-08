//  Copyright (c) 2013  Christos Bergeles and Imperial College London

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

#ifndef HOMOGENEOUSTRANSFORM_H
#define HOMOGENEOUSTRANSFORM_H

#include "screwsInitLibrary.hpp"
#include "screwException.hpp"
#include <Eigen/Eigen>
#include <cfloat>

namespace screws
{
  template<class NumType>
  class Translation;
  template <class NumType>
  class Rotation;
  template <class NumType>
  class Twist;
  
  /*!
   * \class HomogeneousTransform
   * \ingroup libScrews
   * \brief Implements a 4x4 homogeneous transformation matrix along with basic operators.
   * \note Author: Christos Bergeles
   * \date 5th May 2015
   */
  template<class NumType>
  class SCREWS_EXPORT HomogeneousTransform
  {
  public:
    template<class NumTypeTrans> friend class Translation;
    template<class NumTypeRot> friend class Rotation;
    
    /// @brief Create a default homogeneous transformation unit matrix.
    HomogeneousTransform<NumType>()
    {
      _zeroVal = (NumType)0;
      _oneVal = (NumType)1;
    }
    
    /// @brief Create a homogeneous transformation matrix from a rotation and a translation.
    /// @param R the Rotation matrix.
    /// @param T the Translation vector.
    HomogeneousTransform<NumType>(const Rotation<NumType>& R, const Translation<NumType>& T)
    {
      setRotation(R);
      setTranslation(T);
      _zeroVal = (NumType)0;
      _oneVal = (NumType)1;
    }
    
    /// @brief Return the 3x1 translation vector.
    /// @return the 3x1 translation vector.
    Translation<NumType> translation() const
    {
      return _T;
    }
    
    /// @brief Change the translation part of the homogeneous transformation matrix.
    /// @param T the new translation component.
    void setTranslation(const Translation<NumType>& T)
    {
      _T = T;
    }
    
    /// @brief Return the rotational part of the homogeneous transformation matrix.
    /// @return the 3x3 rotational component.
    Rotation<NumType> rotation() const
    {
      return _R;
    }
    
    /// @brief Set the rotational part of the homegeneous transformation matrix.
    /// @param R the new rotational component.
    void setRotation(const Rotation<NumType>& R)
    {
      _R = R;
    }
    
    /// @brief Calculate the twist (log) of the homogeneous transformation matrix.
    /// @return the 4x4 twist skew symmetric matrix.
    Twist<NumType> log() const
    {
      return twist();
    }
    
    /// @brief Calculate the twist (log) of the homogeneous transformation matrix.
    /// @return the 4x4 twist skew symmetric matrix.
    Twist<NumType> twist() const
    {
      return Twist<NumType>(*this);
    }
    
    /// @brief Perform fast inversion of the homogeneous transformation matrix.
    /// @return the inverted homogeneous transformation matrix.
    HomogeneousTransform<NumType> inv() const
    {
      Rotation<NumType> Rinv = _R.inv();
      HomogeneousTransform<NumType> inverted(Rinv, Rinv*((NumType)(-1)*_T));
      return inverted;
    }

    /// @brief Matrix multiplication.
    /// @return the resulting homogeneous transform.
    HomogeneousTransform<NumType> operator*(const HomogeneousTransform<NumType>& H)
    {
      Rotation<NumType> Rnew = _R*H.rotation();
      Translation<NumType> Tnew = _R*H.translation() + _T;
      
      return HomogeneousTransform(Rnew, Tnew);
    }
    
    /// @brief In-place matrix multiplication.
    /// @return the resulting homogeneous transform.
    const HomogeneousTransform<NumType>& operator *=(const HomogeneousTransform<NumType>& H)
    {
      Rotation<NumType> Rnew = _R*H.rotation();
      Translation<NumType> Tnew = _R*H.translation() + _T;

      setRotation(Rnew);
      setTranslation(Tnew);
      
      return *this;
    }
    
    /// @brief Multiplication with a translation.
    /// @return the resulting translation.
    Translation<NumType> operator*(const Translation<NumType>& T)
    {
      return Translation<NumType>(_R*T + _T);
    }
    
    /// @brief Element-by-element exact equality operator.
    /// @return true if all elements are exactly the same.
    bool operator ==(const HomogeneousTransform& H)
    {
      bool equal = true;
      for(int i = 0; i < 4; ++i)
        for(int j = 0; j < 4; ++j)
          equal = equal && ((*this)(i, j) == H(i, j));
      
      return equal;
    }
    
    /// @brief Element-by-element inequality operator.
    /// @return true if any of the elements are not exactly the same.
    bool operator !=(const HomogeneousTransform& H)
    {
      return !(*this == H);
    }

    /// @brief Return the requested element (read).
    /// @param i the index of the row.
    /// @param j the index of the column.
    /// @throw scews::ScrewException for out of bounds indices.
    const NumType& operator () (const unsigned int& i, const unsigned int& j) const
    {
      assert(i >= 0 && i < 4 && j >= 0 && j < 4);
      if (i == 3 && j != 3)
      {
        return _zeroVal;
      }
      else if (i == 3 && j == 3)
      {
        return _oneVal;
      }
      else if (i < 3 && j < 3)
      {
        return _R(i, j);
      }
      else if (i < 3 && j == 3)
      {
        return _T(i);
      }
      else
      {
        screws::ScrewException s("Index out of bounds.", __FILE__, __FUNCTION__, __LINE__);
        throw s;
      }
    }
    
    /// @brief Approximal equality operator, within a given epsilon or system precision.
    /// @param T compared translation.
    /// @param eps desired precision [default: machine precision].
    bool approxEq(const HomogeneousTransform<NumType>& H, double eps = FLT_EPSILON)
    {
      bool approxEq = true;
      for(int i = 0; i < 4; ++i)
      {
        for(int j = 0; j < 4; ++j)
        {
          approxEq = approxEq && (fabs((*this)(i, j) - H(i, j)) < eps);
        }
      }
      
      return approxEq;
    }
    
    /// @brief Perform verification that matrix is indeed in approriate format by checking
    /// the rotational component for validity.
    bool isValid() const
    {
      return _R.isValid();
    }
    
  private:
    
    // The rotation part of the matrix.
    Rotation<NumType> _R;
    // The translation part of the matrix.
    Translation<NumType> _T;
    // One in fixed memory
    NumType _oneVal;
    // Zero in fixed memory
    NumType _zeroVal;
  };
  
  /// @brief Print to stream.
  template <class NumType>
  std::ostream& operator<<(std::ostream& os, const HomogeneousTransform<NumType>& H)
  {
    os << "[" << H(0, 0) << ", " << H(0, 1) << ", " << H(0, 2) << ", " << H(0, 3) << ";\n";
    os <<        H(1, 0) << ", " << H(1, 1) << ", " << H(1, 2) << ", " << H(1, 3) << ";\n";
    os <<        H(2, 0) << ", " << H(2, 1) << ", " << H(2, 2) << ", " << H(2, 3) << ";\n";
    os <<        H(3, 0) << ", " << H(3, 1) << ", " << H(3, 2) << ", " << H(3, 3) << "]";
    
    return os;
  }

  // Convenience names
  using HomogeneousTransformd = HomogeneousTransform < double >;
  using HomogeneousTransformf = HomogeneousTransform < float >;
};

#endif // HOMOGENEOUSTRANSFORM_H
