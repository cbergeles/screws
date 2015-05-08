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

#ifndef TWIST_H
#define TWIST_H

#include "screwsInitLibrary.hpp"
#include "screwException.hpp"
#include <Eigen/Eigen>
#include <cfloat>

namespace screws
{
  template<class NumType>
  class Translation;
  template<class NumType>
  class Rotation;
  template<class NumType>
  class HomogeneousTransform;
  template<class NumType>
  class Skew;
  template<class NumType>
  class Vector6;

  template<typename NumType>
  using TwistCoordinates = Vector6 < NumType >;

  using TwistCoordinatesd = Vector6d;
  using TwistCoordinatesf = Vector6f;

  /*!
   * \class Twist
   * \ingroup libScrews
   * \brief Implements a 4x4 twist matrix with basic functionality such as exponents, logs etc.
   * \note Author: Christos Bergeles
   * \date 8th May 2015
   */
  template<class NumType>
  class SCREWS_EXPORT Twist
  {
  public:
    template<class NumTypeHomo> friend class HomogeneousTransform;

    /// @brief Initialise with zeros. This corresponds to the identity homogeneous transformation matrix.
    Twist()
    {
      _zeroVal = (NumType)0;
    }

    /// @brief Create a twist by taking the logarithm of a homogeneous transformation matrix.
    /// @param HT a homogeneous transformation matrix.
    Twist(const HomogeneousTransform<NumType>& HT)
    {
      Eigen::Matrix<NumType, 3, 3> Ainv;

      // Create the skew matrix from the rotation part of the homogeneous transform
      _skew = Skew<NumType>(HT.rotation());

      // p. 414 from Sastry
      Translation<NumType> trans = HT.translation();

      // Get the norm of the rotation
      NumType skewNorm = _skew.norm();

      // Pure rotation
      if ( fabs(trans.norm()) < 1e-10)
      {
        Ainv = Eigen::Matrix<NumType, 3, 3>::Zero();
      }
      // Pure translation
      else if (fabs(skewNorm) < 1e-10)
      {
        Ainv = Eigen::Matrix<NumType, 3, 3>::Identity();
      }
      else
      {
        Ainv = Eigen::Matrix<double, 3, 3>::Identity() +
          _skew._data/2 +
          (2*sin(skewNorm) - skewNorm*(1 + cos(skewNorm)))/(2*skewNorm*skewNorm*sin(skewNorm))*_skew._data*_skew._data;

      }
      _velocity = Translation<NumType>(
            Ainv(0, 0)*trans(0) + Ainv(0, 1)*trans(1) + Ainv(0, 2)*trans(2),
            Ainv(1, 0)*trans(0) + Ainv(1, 1)*trans(1) + Ainv(1, 2)*trans(2),
            Ainv(2, 0)*trans(0) + Ainv(2, 1)*trans(1) + Ainv(2, 2)*trans(2)
            );

      _zeroVal = (NumType)0;
    }

    /// @brief Create a twist using a twistcoordinates vector.
    /// @param V the twist coordinates.
    /// @note V(0)-V(2) correspond to the velocity, and V(3)-V(5) to the rotation.
    Twist(const TwistCoordinates<NumType>& V)
    {
      _velocity(0) = V(0);
      _velocity(1) = V(1);
      _velocity(2) = V(2);

      _skew = Skew<NumType>(Translation<NumType>(V(3), V(4), V(5)));

      _zeroVal = (NumType)0;
    }

    /// @brief Create a twist using the 6 twist coordinates.
    /// @param t0-t6: the twist coordinates.
    Twist(const NumType& t0, const NumType& t1, const NumType& t2,
          const NumType& t3, const NumType& t4, const NumType& t5)
    {
      _velocity(0) = t0;
      _velocity(1) = t1;
      _velocity(2) = t2;

      _skew = Skew<NumType>(Translation<NumType>(t3, t4, t5));

      _zeroVal = (NumType)0;
    }

    /// Default destructor.
    ~Twist()
    {

    }

    /// @brief Calculate and return the pitch of the twist.
    /// @return the pitch of the twist.
    NumType pitch() const
    {
      NumType p;

      Translation<NumType> omega = _skew.coordinates();
      NumType normOmega = omega.norm();

      if (fabs(normOmega) < 1e-10)
      {
        p = (NumType)1e+10;
      }
      else
      {
        p = (omega.dot(_velocity))/(normOmega*normOmega);
      }
      return p;
    }

    /// @brief Calculate and return the axis of the twist.
    /// @return a TwistCoordinates vector w containing the axis of the twist, with  w(0:2) + \lambda*w(3:5), \lambda\in\mathcal{R}.
    TwistCoordinates<NumType> axis() const
    {
      TwistCoordinates<NumType> ax;
      Translation<NumType> omega = _skew.coordinates();
      NumType normOmega = omega.norm();

      if (fabs(normOmega) < 1e-10)
      {
        ax._v1 = _velocity;
      }
      else
      {
        Eigen::Matrix<NumType, 3, 1, 0, 3, 1> m = _skew._data*_velocity._data/(normOmega*normOmega);
        ax._v0.setData(m(0), m(1), m(2));
        ax._v1 = omega;
      }

      return ax;
    }

    /// @brief Calculate and return the 6x1 twist coordinates.
    /// @return the TwistCoordinates vector.
    /// @note V(0)-V(2) contain the velocity of the twist, and V(3)-V(5) the rotation.
    TwistCoordinates<NumType> coordinates() const
    {
      return TwistCoordinates<NumType>(_velocity, _skew.coordinates());
    }

    /// @brief Calculate and return the norm of the twist.
    /// @return the norm of the twist.
    NumType norm() const
    {
      NumType omegaNorm = _skew.norm();
      if (fabs(omegaNorm) < 1e-10)
      {
        return _velocity.norm();
      }
      else
      {
        return omegaNorm;
      }
    }

    /// @brief Create a homogeneous transformation matrix by taking the exponential of the twist.
    /// @param theta the magnitude of the rotation.
    /// @return the homogeneous transformation matrix.
    HomogeneousTransform<NumType> exp(const NumType& theta = (NumType)1) const
    {
      // p. 413 Sastry
      NumType omegaNorm = _skew.angle();

      if (fabs(omegaNorm) < 1e-10 || theta == (NumType)0) // pure translation or identity matrix
      {
        return HomogeneousTransform<NumType>(Rotation<NumType>(), _velocity*theta);
      }
      else
      {
        Eigen::Matrix<NumType, 3, 3> temp = _skew._data/(theta*omegaNorm*omegaNorm);
        Eigen::Matrix<NumType, 3, 3> temp2 = temp*_skew._data/omegaNorm;

        Rotation<NumType> R = _skew.exp(theta);
        Eigen::Matrix<NumType, 3, 3> A =
            Eigen::Matrix<NumType, 3, 3>::Identity() +
            temp*(1 - cos(omegaNorm*theta)) +
            temp2*(omegaNorm*theta - sin(omegaNorm*theta));

        Eigen::Matrix<NumType, 3, 1, 0, 3, 1> v = A*_velocity._data*theta;

        return HomogeneousTransform<NumType>(R, Translation<NumType>(v(0), v(1), v(2)));
      }
    }

    /// @brief Perform verification that matrix is indeed in approriate format by checking the Skew component.
    bool isValid() const
    {
      return _skew.isValid();
    }

    /// @brief Returns the skew symmetric (rotation) part of the twist.
    /// @return the skew symmetric part of the twist.
    Skew<NumType> skew() const
    {
      return _skew;
    }

    /// @brief Returns the velocity part of the twist.
    /// @return the velocity (3x1 vector) part of the twist.
    Translation<NumType> velocity() const
    {
      return _velocity;
    }

    /// @brief Element-by-element addition of twist matrices.
    /// @note This operation corresponds to multiplication of the respective homogeneous transforms.
    Twist<NumType> operator +(const Twist<NumType>& T) const
    {
      return Twist<NumType>(T.coordinates() + coordinates());
    }

    /// @brief Element-by-element in-place addition of skew matrices.
    /// @note This operation corresponds to multiplication of the respective rotations.
    const Twist<NumType>& operator +=(const Twist<NumType>& T)
    {
      Twist<NumType> newTwist(T.coordinates() + coordinates());

      _skew = newTwist._skew;
      _velocity = newTwist._velocity;

      return *this;
    }

    /// @brief Element-by-element exact equality operator.
    /// @return true if all elements are exactly equal
    bool operator ==(const Twist<NumType>& T) const
    {
      bool valid = (T.skew() == _skew && T.velocity() == _velocity);

      return valid;
    }

    /// @brief Inequality operator.
    /// @return true if one of the element-by-element comparisons return true. Otherwise, false.
    bool operator !=(const Twist<NumType>& T) const
    {
      if (T == *this)
      {
        return false;
      }
      else
      {
        return true;
      }
    }

    /// @brief Approximal equality operator, within a given epsilon or system precision.
    /// @param T compared twist matrix.
    /// @param eps desired precision [default: machine precision].
    bool approxEq(const Twist<NumType>& T, double eps = FLT_EPSILON) const
    {
      if (T.skew().approxEq(_skew, eps) && T.velocity().approxEq(_velocity, eps))
      {
        return true;
      }
      else
      {
        return false;
      }
    }

    /// @brief Return the requested element (read).
    /// @param i the index of the row.
    /// @param j the index of the column.
    /// @throw scews::ScrewException for out of bounds indices.
    const NumType& operator () (const unsigned int& i, const unsigned int& j) const
    {
      assert(i >= 0 && i < 4 && j >= 0 && j < 4);
      if (i == 3)
      {
        return _zeroVal;
      }
      else if (j < 3)
      {
        return _skew(i, j);
      }
      else if (j == 3)
      {
        return _velocity(i);
      }
      else
      {
        screws::ScrewException s("Index out of bounds.", __FILE__, __FUNCTION__, __LINE__);
        throw s;
      }
    }

  protected:

    // Set all to zero.
    void resetData()
    {
      _skew = Skew<NumType>();
      _velocity = Translation<NumType>();
    }

    Skew<NumType> _skew;
    Translation<NumType> _velocity;
    // Zero in fixed memory
    NumType _zeroVal;
  };

  /// @brief Print to stream.
  template <class NumType>
  std::ostream& operator<<(std::ostream& os, const Twist<NumType>& Tw)
  {
    os << "[" << Tw(0, 0) << ", " << Tw(0, 1) << ", " << Tw(0, 2) << ", " << Tw(0, 3) << ";\n";
    os <<        Tw(1, 0) << ", " << Tw(1, 1) << ", " << Tw(1, 2) << ", " << Tw(1, 3) << ";\n";
    os <<        Tw(2, 0) << ", " << Tw(2, 1) << ", " << Tw(2, 2) << ", " << Tw(2, 3) << ";\n";
    os <<        Tw(3, 0) << ", " << Tw(3, 1) << ", " << Tw(3, 2) << ", " << Tw(3, 3) << "]";

    return os;
  }

  // Convenience names
  using Twistd = Twist < double >;
  using Twistf = Twist < float >;
};


#endif // TWIST_H
