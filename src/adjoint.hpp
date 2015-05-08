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
#ifndef ADJOINT_H
#define ADJOINT_H

#include "screwsInitLibrary.h"

#include "armadillo"

class Translation;
class Rotation;
class HomogeneousTransform;
class Skew;
class Twist;

/*!
 * \class Adjoint
 * \ingroup libScrews
 * \brief Implements a 6x6 adjoint matrix with basic functionality.
 * \note Author: Christos Bergeles
 * \date 2013-07-02
 */
class SCREWS_EXPORT Adjoint
{
public:

  /*!
   * \brief Create a zero 6x6 matrix for the Adjoint.
   */
  explicit Adjoint();
  /*!
   * \brief Create an adjoint matrix from a Rotation and a Translation.
   * @param R the rotational component.
   * @param T the translational component.
   */
  explicit Adjoint(const Rotation& R, const Translation& T);
  /*!
   * \brief Create an adjoint for the given homogeneous transformation matrix.
   * @param H the homogeneous transformation matrix.
   */
  explicit Adjoint(const HomogeneousTransform& H);

  /// Default destructor.
  ~Adjoint();

  /*!
   * \brief Accessor function for ease of use.
   * @param i the i-th row of the matrix.
   * @param j the j-th row of the matrix.
   * \note No boundary check is performed.
   */
  double at(const int& i, const int& j);

  /*!
   * \brief Calculates the inverse of the adjoint in a computationally efficient way.
   */
  Adjoint inv() const;

  /*!
   * \brief Handle multiplication with a twist coordinates vector (6x1).
   * @param ksi the 6x1 coordinates vector.
   * \note No dimension check for ksi is performed.
   */
  arma::colvec::fixed<6> operator *(const arma::vec::fixed<6>& ksi);
  /*!
   * \brief Handle multiplication with a jacobian.
   * @param jacobian the 6xN jacobian matrix.
   * \note No dimension check for jacobian is performed.
   */
  arma::mat operator *(const arma::mat& jacobian);

  bool operator ==(const Adjoint& A);

  void print() const;

private:
  void construct(const Rotation& R, const Translation& T);
  arma::mat::fixed<6, 6> _data;
  Translation _T;
  Rotation _R;
};

#endif // ADJOINT_H
