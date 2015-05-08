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

#include "screws.h"

void
Adjoint::construct(const Rotation& R, const Translation& T)
{
  _data = arma::zeros<arma::mat>(6, 6);
  arma::colvec::fixed<3> v;

  v(0) = T.at(0); v(1) = T.at(1); v(2) = T.at(2);
  Skew S(v);
  arma::mat::fixed<3, 3> R2 = S._data*R._data;

  for(int i = 0; i < 3; ++i)
    for(int j = 0; j < 3; ++j)
    {
      _data(i, j + 3) = R2(i, j);
      _data(i, j) = _data(i + 3, j + 3) = R.at(i, j);
    }
  _R = R;
  _T = T;
}

Adjoint::~Adjoint()
{
}

Adjoint::Adjoint(const Rotation& R, const Translation& T)
{
  construct(R, T);
}

Adjoint::Adjoint(const HomogeneousTransform& H)
{
  construct(H.getRotation(), H.getTranslation());
}

Adjoint::Adjoint()
{
  construct(Rotation(), Translation());
}

double
Adjoint::at(const int& i, const int& j)
{
  return _data(i, j);
}

bool
Adjoint::operator ==(const Adjoint& A)
{
  bool isEqual = true;
  for(int i = 0; i < 6; ++i)
    for(int j = 0; j < 6; ++j)
      isEqual = (isEqual && (fabs(_data(i, j) - A._data(i, j)) < 1e-10));

  return isEqual;
}

Adjoint
Adjoint::inv() const
{
  // The inverse of an adjoint is the adjoint of the inverse
  HomogeneousTransform H(_R, _T);

  return Adjoint(H.inv());
}

void
Adjoint::print() const
{
  _data.print("A = ");
}

arma::vec::fixed<6>
Adjoint::operator *(const arma::vec::fixed<6>& ksi)
{
  return _data*ksi;
}

arma::mat
Adjoint::operator *(const arma::mat& jacobian)
{
  return _data*jacobian;
}
