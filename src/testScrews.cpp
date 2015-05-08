#include <iostream>
#include <time.h>

#define SHOW_PRINT_OUTS false
#define TEST_VECTOR6 true
#define TEST_TRANSLATIONS true
#define TEST_ROTATIONS true
#define TEST_HOMOGENEOUS_TRANSFORMS true
#define TEST_SKEWS true
#define TEST_TWISTS true

#include "translation.hpp"
#include "rotation.hpp"
#include "homogeneousTransform.hpp"
#include "skew.hpp"
#include "vector6.hpp"
#include "twist.hpp"

void testVector6()
{
  if (SHOW_PRINT_OUTS) std::cout << " == VECTOR6 == " << std::endl;
  int testIdx = 1;

  screws::Vector3<double> T1rand((double)rand()/RAND_MAX,
                                 (double)rand()/RAND_MAX,
                                 (double)rand()/RAND_MAX);

  screws::Vector3<double> T2rand((double)rand()/RAND_MAX,
                                 (double)rand()/RAND_MAX,
                                 (double)rand()/RAND_MAX);

  screws::Vector6<double> Vrand(T1rand, T2rand);
  assert(Vrand(0) == T1rand(0) && Vrand(1) == T1rand(1) && Vrand(2) == T1rand(2));
  assert(Vrand(3) == T2rand(0) && Vrand(4) == T2rand(1) && Vrand(5) == T2rand(2));

  screws::Vector6<double> Vzero;
  assert(Vzero(0) == 0 && Vzero(1) == 0 && Vzero(2) == 0 &&
         Vzero(3) == 0 && Vzero(4) == 0 && Vzero(5) == 0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Default constructor passed." << std::endl;

  screws::Vector6<double> Vfixed(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
  assert(Vfixed(0) == 1.0 && Vfixed(1) == 2.0 && Vfixed(2) == 3.0 &&
         Vfixed(3) == 4.0 && Vfixed(4) == 5.0 && Vfixed(5) == 6.0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Custom constructor passed." << std::endl;

  screws::Vector6<double> V2 = Vfixed;
  screws::Vector6<double> V3 = Vfixed + Vrand;
  assert(V3(0) == Vfixed(0) + Vrand(0) &&
         V3(1) == Vfixed(1) + Vrand(1) &&
         V3(2) == Vfixed(2) + Vrand(2) &&
         V3(3) == Vfixed(3) + Vrand(3) &&
         V3(4) == Vfixed(4) + Vrand(4) &&
         V3(5) == Vfixed(5) + Vrand(5));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element addition passed." << std::endl;

  V3 += V2;
  assert(fabs(V3(0) - 2*V2(0) - Vrand(0)) < FLT_EPSILON &&
         fabs(V3(1) - 2*V2(1) - Vrand(1)) < FLT_EPSILON &&
         fabs(V3(2) - 2*V2(2) - Vrand(2)) < FLT_EPSILON &&
         fabs(V3(3) - 2*V2(3) - Vrand(3)) < FLT_EPSILON &&
         fabs(V3(4) - 2*V2(4) - Vrand(4)) < FLT_EPSILON &&
         fabs(V3(5) - 2*V2(5) - Vrand(5)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-element addition passed." << std::endl;

  V3 = V3 - V2;
  assert(fabs(V3(0) - V2(0) - Vrand(0)) < FLT_EPSILON &&
         fabs(V3(1) - V2(1) - Vrand(1)) < FLT_EPSILON &&
         fabs(V3(2) - V2(2) - Vrand(2)) < FLT_EPSILON &&
         fabs(V3(3) - V2(3) - Vrand(3)) < FLT_EPSILON &&
         fabs(V3(4) - V2(4) - Vrand(4)) < FLT_EPSILON &&
         fabs(V3(5) - V2(5) - Vrand(5)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element subtraction passed." << std::endl;

  V3 -= V2;
  assert(fabs(V3(0) - Vrand(0)) < FLT_EPSILON &&
         fabs(V3(1) - Vrand(1)) < FLT_EPSILON &&
         fabs(V3(2) - Vrand(2)) < FLT_EPSILON &&
         fabs(V3(3) - Vrand(3)) < FLT_EPSILON &&
         fabs(V3(4) - Vrand(4)) < FLT_EPSILON &&
         fabs(V3(5) - Vrand(5)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-elemend subtraction passed." << std::endl;

  assert(V3.approxEq(Vrand));
  assert(V3 != (Vrand + 1));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element equality passed." << std::endl;

  V3 = V2 + 4;
  assert(V3(0) == V2(0) + 4 &&
         V3(1) == V2(1) + 4 &&
         V3(2) == V2(2) + 4 &&
         V3(3) == V2(3) + 4 &&
         V3(4) == V2(4) + 4 &&
         V3(5) == V2(5) + 4);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Addition of value passed." << std::endl;

  V3 += 4;
  assert(V3(0) == V2(0) + 8 &&
         V3(1) == V2(1) + 8 &&
         V3(2) == V2(2) + 8 &&
         V3(3) == V2(3) + 8 &&
         V3(4) == V2(4) + 8 &&
         V3(5) == V2(5) + 8);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place addition of value passed." << std::endl;

  V3 = V2 - 4;
  assert(V3(0) == V2(0) - 4 &&
         V3(1) == V2(1) - 4 &&
         V3(2) == V2(2) - 4 &&
         V3(3) == V2(3) - 4 &&
         V3(4) == V2(4) - 4 &&
         V3(5) == V2(5) - 4);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Subtraction of value passed." << std::endl;

  V3 -= 4;
  assert(V3 == V2 - 8);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place subtraction of value passed." << std::endl;

  V3 = V2*2;
  assert(V3 == (V2 + V2));
  V3 = 2.0*V2;
  assert(V3 == (V2 + V2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Multiplication by value passed." << std::endl;

  V3 *= 2;
  assert(V3 == (V2 + V2 + V2 + V2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place multiplication by value passed." << std::endl;

  V3 = V2*V2;
  assert(V3(0) == V2(0)*V2(0) &&
         V3(1) == V2(1)*V2(1) &&
         V3(2) == V2(2)*V2(2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element multiplication passed." << std::endl;

  V3 *= V2;
  assert(V3 == (V2*V2*V2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-element multiplication passed." << std::endl;

  V3 = V2/V2;
  assert(V3(0) == 1 && V3(1) == 1 && V3(2) == 1);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element division passed." << std::endl;

  V2(0) = 0;
  try
  {
    V3 = V3/V2;
    exit(1);
  }
  catch(screws::ScrewException s)
  {
    if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Division by zero passed: " << s.what() << std::endl;
  }

  V2(0) = 2.0;

  V3 = V2; V3 /= V2;
  assert(V3(0) == 1 && V3(1) == 1 && V3(2) == 1 &&
         V3(3) == 1 && V3(4) == 1 && V3(5) == 1);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-element division passed." << std::endl;

  V3 = V3/2.0;
  assert(V3(0) == 0.5 && V3(1) == 0.5 && V3(2) == 0.5 &&
      V3(3) == 0.5 && V3(4) == 0.5 && V3(5) == 0.5);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Division by value passed." << std::endl;

  V3 /= 2.0;
  assert(V3(0) = 0.25 && V3(1) == 0.25 && V3(2) == 0.25 && V3(3) == 0.25 && V3(4) == 0.25 && V3(5) == 0.25);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place division by value passed." << std::endl;

  V3(0) = 100;
  assert(V3(0) == 100);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element set passed." << std::endl;

  assert(fabs(V3.norm()*V3.norm() -
              (V3(0)*V3(0) + V3(1)*V3(1) + V3(2)*V3(2) +
               V3(3)*V3(3) + V3(4)*V3(4) + V3(5)*V3(5))) < 1e-10);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Norm passed." << std::endl;

  double n = Vrand.norm();
  screws::Vector6<double> V4norm = Vrand.normalised();
  screws::Vector6<double> V4normnorm = V4norm.normalised();

  assert(fabs(V4norm.norm() - 1) < FLT_EPSILON);
  assert(fabs(V4normnorm.norm() - 1) < FLT_EPSILON);
  assert(V4norm.approxEq(V4normnorm));
  assert(Vrand.approxEq(n*V4norm));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Normalisation passed." << std::endl;

  V3 = 0.0*V3;
  try
  {
    V3 = V3.normalised();
    exit(1);
  }
  catch(screws::ScrewException s)
  {
    if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Normalisation of zero vector passed: " << s.what() << std::endl;
  }

  assert(fabs(Vrand.norm()*Vrand.norm() - Vrand.dot(Vrand)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Dot product passed." << std::endl;
}

void testTranslations()
{
  if (SHOW_PRINT_OUTS) std::cout << " == TRANSLATION == " << std::endl;
  int testIdx = 1;

  screws::Translation<double> Trand((double)rand()/RAND_MAX,
                                    (double)rand()/RAND_MAX,
                                    (double)rand()/RAND_MAX);

  screws::Translation<double> T1;
  assert(T1(0) == 0 && T1(1) == 0 && T1(2) == 0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Default constructor passed." << std::endl;

  screws::Translation<double> T2(1.0, 2.0, 3.0);
  assert(T2(0) == 1.0 && T2(1) == 2.0 && T2(2) == 3.0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Custom constructor passed." << std::endl;

  screws::Translation<double> T3 = T2 + Trand;
  assert(T3(0) == T2(0) + Trand(0) &&
         T3(1) == T2(1) + Trand(1) &&
         T3(2) == T2(2) + Trand(2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element addition passed." << std::endl;

  T3 += T2;
  assert(fabs(T3(0) - 2*T2(0) - Trand(0)) < FLT_EPSILON &&
         fabs(T3(1) - 2*T2(1) - Trand(1)) < FLT_EPSILON &&
         fabs(T3(2) - 2*T2(2) - Trand(2)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-element addition passed." << std::endl;

  T3 = T3 - T2;
  assert(fabs(T3(0) - T2(0) - Trand(0)) < FLT_EPSILON &&
         fabs(T3(1) - T2(1) - Trand(1)) < FLT_EPSILON &&
         fabs(T3(2) - T2(2) - Trand(2)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element subtraction passed." << std::endl;

  T3 -= T2;
  assert(fabs(T3(0) - Trand(0)) < FLT_EPSILON &&
         fabs(T3(1) - Trand(1)) < FLT_EPSILON &&
         fabs(T3(2) - Trand(2)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-elemend subtraction passed." << std::endl;

  assert(T3.approxEq(Trand));
  assert(T3 != (Trand + 1));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element equality passed." << std::endl;

  T3 = T2 + 4;
  assert(T3(0) == T2(0) + 4 &&
         T3(1) == T2(1) + 4 &&
         T3(2) == T2(2) + 4);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Addition of value passed." << std::endl;

  T3 += 4;
  assert(T3(0) == T2(0) + 8 &&
         T3(1) == T2(1) + 8 &&
         T3(2) == T2(2) + 8);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place addition of value passed." << std::endl;

  T3 = T2 - 4;
  assert(T3(0) == T2(0) - 4 &&
         T3(1) == T2(1) - 4 &&
         T3(2) == T2(2) - 4);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Subtraction of value passed." << std::endl;

  T3 -= 4;
  assert(T3 == T2 - 8);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place subtraction of value passed." << std::endl;

  T3 = T2*2;
  assert(T3 == (T2 + T2));
  T3 = 2.0*T2;
  assert(T3 == (T2 + T2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Multiplication by value passed." << std::endl;

  T3 *= 2;
  assert(T3 == (T2 + T2 + T2 + T2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place multiplication by value passed." << std::endl;

  T3 = T2*T2;
  assert(T3(0) == T2(0)*T2(0) &&
         T3(1) == T2(1)*T2(1) &&
         T3(2) == T2(2)*T2(2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element multiplication passed." << std::endl;

  T3 *= T2;
  assert(T3 == (T2*T2*T2));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-element multiplication passed." << std::endl;

  T3 = T2/T2;
  assert(T3(0) == 1 && T3(1) == 1 && T3(2) == 1);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element-by-element division passed." << std::endl;

  T2(0) = 0;
  try
  {
    T3 = T3/T2;
    exit(1);
  }
  catch(screws::ScrewException s)
  {
    if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Division by zero passed: " << s.what() << std::endl;
  }

  T2(0) = 2.0;

  T3 = T2; T3 /= T2;
  assert(T3(0) == 1 && T3(1) == 1 && T3(2) == 1);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place element-by-element division passed." << std::endl;

  T3 = T3/2.0;
  assert(T3(0) = 0.5 && T3(1) == 0.5 && T3(2) == 0.5);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Division by value passed." << std::endl;

  T3 /= 2.0;
  assert(T3(0) = 0.25 && T3(1) == 0.25 && T3(2) == 0.25);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place division by value passed." << std::endl;

  T3(0) = 100;
  assert(T3(0) == 100);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Element set passed." << std::endl;

  assert(T3.norm()*T3.norm() == (T3(0)*T3(0) + T3(1)*T3(1) + T3(2)*T3(2)));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Norm passed." << std::endl;

  screws::Translation<double> T4((double)rand()/RAND_MAX,
                                 (double)rand()/RAND_MAX,
                                 (double)rand()/RAND_MAX);
  double n = T4.norm();
  screws::Translation<double> T4norm = T4.normalised();
  screws::Translation<double> T4normnorm = T4norm.normalised();

  assert(fabs(T4norm.norm() - 1) < FLT_EPSILON);
  assert(fabs(T4normnorm.norm() - 1) < FLT_EPSILON);
  assert(T4norm.approxEq(T4normnorm));
  assert(T4.approxEq(n*T4norm));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Normalisation passed." << std::endl;

  T3 = 0.0*T3;
  try
  {
    T3 = T3.normalised();
    exit(1);
  }
  catch(screws::ScrewException s)
  {
    if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Normalisation of zero vector passed: " << s.what() << std::endl;
  }

  T3(0) = 1.0; T3(1) = 0.0; T3(2) = 0.0;
  T4(0) = 0.0; T4(1) = 1.0; T4(2) = 0.0;
  assert(T3.cross(T4) == screws::Translationd(0.0, 0.0, 1.0));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Cross product passed." << std::endl;

  assert(fabs(Trand.norm()*Trand.norm() - Trand.dot(Trand)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Dot product passed." << std::endl;
}

void testTwists()
{
  int testIdx = 1;
  static int testRun = 0;
  testRun++;

  if (SHOW_PRINT_OUTS) std::cout << " == TWIST == " << std::endl;

  screws::Twistd TwistZero;
  for(int i = 0; i < 4; ++i)
  {
    for(int j = 0; j < 4; ++j)
    {
      assert(TwistZero(i, j) == 0);
    }
  }
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Default constructor test passed." << std::endl;

  screws::HomogeneousTransformd Hx(screws::Rotationd('x', 1.999*M_PI*(double)rand()/RAND_MAX),
                                   screws::Translationd((double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX));
  screws::HomogeneousTransformd Hy(screws::Rotationd('y', 1.999*M_PI*(double)rand()/RAND_MAX),
                                   screws::Translationd((double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX));
  screws::HomogeneousTransformd Hz(screws::Rotationd('z', 1.999*M_PI*(double)rand()/RAND_MAX),
                                   screws::Translationd((double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX));
  screws::HomogeneousTransformd HxRot(screws::Rotationd('x', 1.999*M_PI*(double)rand()/RAND_MAX),
                                   screws::Translationd());
  screws::HomogeneousTransformd HyRot(screws::Rotationd('y', 1.999*M_PI*(double)rand()/RAND_MAX),
                                   screws::Translationd());
  screws::HomogeneousTransformd HzRot(screws::Rotationd('z', 1.999*M_PI*(double)rand()/RAND_MAX),
                                   screws::Translationd());
  screws::HomogeneousTransformd HTra(screws::Rotationd(),
                                   screws::Translationd((double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX,
                                                        (double)rand()/RAND_MAX));

  screws::HomogeneousTransformd Hrand(screws::Rotationd(
                                        screws::Vector3d(
                                          (double)rand()/RAND_MAX,
                                          (double)rand()/RAND_MAX,
                                          (double)rand()/RAND_MAX),
                                        1.999*M_PI*(double)rand()/RAND_MAX),
                                      screws::Translationd(
                                        (double)rand()/RAND_MAX,
                                        (double)rand()/RAND_MAX,
                                        (double)rand()/RAND_MAX));
  screws::HomogeneousTransformd HrandRot(screws::Rotationd(
                                        screws::Vector3d(
                                          (double)rand()/RAND_MAX,
                                          (double)rand()/RAND_MAX,
                                          (double)rand()/RAND_MAX),
                                        1.999*M_PI*(double)rand()/RAND_MAX),
                                      screws::Translationd());

  screws::HomogeneousTransformd HrandTra(screws::Rotationd(),
                                      screws::Translationd(
                                        (double)rand()/RAND_MAX,
                                        (double)rand()/RAND_MAX,
                                        (double)rand()/RAND_MAX));


  screws::Twistd Twx(Hx);
  screws::Twistd Twy(Hy);
  screws::Twistd Twz(Hz);
  screws::Twistd Twrand(Hrand);
  screws::Twistd TwxRot(HxRot);
  screws::Twistd TwyRot(HyRot);
  screws::Twistd TwzRot(HzRot);
  screws::Twistd TwTra(HTra);
  screws::Twistd TwrandRot(HrandRot);
  screws::Twistd TwrandTra(HrandTra);

  assert(Twx.skew().exp().approxEq(Hx.rotation(), 1e-5));
  assert(Twx.approxEq(Hx.twist()));
  assert(Twy.skew().exp().approxEq(Hy.rotation(), 1e-5));
  assert(Twy.approxEq(Hy.twist()));
  assert(Twz.skew().exp().approxEq(Hz.rotation(), 1e-5));
  assert(Twz.approxEq(Hz.twist()));

  // std::cout << TwrandRot.skew().exp() << std::endl;
  // std::cout << HrandRot.rotation() << std::endl;
  assert(Twrand.skew().exp().approxEq(Hrand.rotation(), 1e-5));
  assert(Twrand.approxEq(Hrand.log()));

  assert(TwxRot.skew().exp().approxEq(HxRot.rotation(), 1e-5));
  assert(TwxRot.approxEq(HxRot.twist()));
  assert(TwyRot.skew().exp().approxEq(HyRot.rotation(), 1e-5));
  assert(TwyRot.approxEq(HyRot.twist()));
  assert(TwzRot.skew().exp().approxEq(HzRot.rotation(), 1e-5));
  assert(TwzRot.approxEq(HzRot.twist()));

  assert(TwTra.skew().exp().approxEq(HTra.rotation(), 1e-5));
  assert(TwTra.approxEq(HTra.twist()));

  // std::cout << TwrandRot.skew().exp() << std::endl;
  // std::cout << HrandRot.rotation() << std::endl;
  assert(TwrandRot.skew().exp().approxEq(HrandRot.rotation(), 1e-5));
  assert(TwrandRot.approxEq(HrandRot.twist()));
  assert(TwrandTra.skew().exp().approxEq(HrandTra.rotation(), 1e-5));
  assert(TwrandTra.approxEq(HrandTra.twist()));

  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Conversion of skew part to rotation generator test passed." << std::endl;

  assert(TwxRot.norm() == TwxRot.skew().norm());
  assert(TwyRot.norm() == TwyRot.skew().norm());
  assert(TwzRot.norm() == TwzRot.skew().norm());
  assert(TwrandRot.norm() == TwrandRot.skew().norm());

  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Norm of pure rotations test passed." << std::endl;

  screws::Skewd SxRot = HxRot.rotation().skew();
  assert(TwxRot.coordinates()(3) == SxRot.coordinates()(0));
  assert(TwxRot.coordinates()(4) == SxRot.coordinates()(1));
  assert(TwxRot.coordinates()(5) == SxRot.coordinates()(2));

  assert(Twrand + Twrand == Twrand + Twrand);
  assert(Twrand + Twrand != Twrand);

  Twrand.pitch();
  Twrand.axis();

}

void testSkews()
{
  static int testRun = 0;
  testRun++;

  if (SHOW_PRINT_OUTS) std::cout << " == SKEW == " << std::endl;
  int testIdx = 1;

  screws::Vector3d randAxis((double)rand()/RAND_MAX,
                            (double)rand()/RAND_MAX,
                            (double)rand()/RAND_MAX);
  if (testRun % 200 == 0)
  {
    randAxis(0) = 0;
  }
  if (testRun % 100 == 0)
  {
    randAxis(1) = 0;
  }
  randAxis = randAxis.normalised();
  double randAngle = (1.9999*M_PI*rand())/RAND_MAX;
  screws::Rotationd Rrand(randAxis, randAngle);

  screws::Skewd Szero;
  assert(Szero(0, 0) == 0 && Szero(0, 1) == 0 && Szero(0, 2) == 0 &&
         Szero(1, 0) == 0 && Szero(1, 1) == 0 && Szero(1, 2) == 0 &&
         Szero(2, 0) == 0 && Szero(2, 1) == 0 && Szero(2, 2) == 0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Zero constructor test passed." << std::endl;

  // Test x, y, z
  screws::Skewd Sx(screws::Vector3d(randAngle, 0, 0));
  screws::Skewd Sy(screws::Vector3d(0, randAngle, 0));
  screws::Skewd Sz(screws::Vector3d(0, 0, randAngle));
  assert(Sx.exp().approxEq(screws::Rotationd('x', randAngle)));
  assert(Sy.exp().approxEq(screws::Rotationd('y', randAngle)));
  assert(Sz.exp().approxEq(screws::Rotationd('z', randAngle)));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Constructor for basic orientations test passed." << std::endl;

  screws::Skewd SrandAxis(randAxis);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Constructor by vector test passed." << std::endl;

  screws::Skewd SrandRot(Rrand);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Constructor by rotation test passed." << std::endl;

  assert(SrandRot.norm() == SrandRot.angle());
  assert(SrandAxis.norm() == SrandAxis.angle());
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Norm and angle test passed." << std::endl;

  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      assert(SrandRot(i, j) == -1*SrandRot.transpose()(i, j));
    }
  }
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Transpose test passed." << std::endl;

  if (!(SrandRot.exp().approxEq(Rrand, 1e-5)))
  {
    std::cout << "SrandRot.exp = " << SrandRot.exp() << std::endl;
    std::cout << "Rrand = " << Rrand << std::endl;
    std::cout << "SrandRot.angle = " << SrandRot.angle() << std::endl;
    std::cout << "randAngle = " << randAngle << std::endl;
  }

  assert(SrandAxis.exp(randAngle).approxEq(Rrand, 1e-5));
  assert(SrandAxis.exp(0.0).approxEq(screws::Rotation<double>(), 1e-5));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Exponential test passed." << std::endl;

  // This is because the sign of the axis can be inverted due to the angle
  if ( (SrandRot.axis().approxEq(-1.0*SrandAxis.axis())) &&
       (fabs(SrandRot.angle() - 2*M_PI + randAngle)) < 1e-5)
  {
    assert(true);
    assert(SrandRot.axis().approxEq((-1.0)*randAxis));
  }
  else
  {
    assert(SrandRot.axis().approxEq(SrandAxis.axis()));
    assert(fabs(SrandRot.angle() - randAngle) < 1e-3);
    assert(SrandRot.axis().approxEq(randAxis));
  }

  assert(fabs(SrandRot.normalised().angle() - 1.0) < 1e-5); // 1e-5 corresponds to 0.00572957795 degrees
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Normalisation test passed." << std::endl;

  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Angle calculation test passed." << std::endl;

  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Axis calculation test passed." << std::endl;

  screws::Skewd Srand2 = SrandRot + SrandRot;
  assert(Srand2.axis().approxEq(SrandRot.axis()) &&
         Srand2.angle() == 2.0*SrandRot.angle());
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Addition test passed." << std::endl;

  Srand2 += SrandRot;
  assert(Srand2.axis().approxEq(SrandRot.axis()));
  assert(fabs(Srand2.angle() - 3.0*SrandRot.angle()) < 1e-5);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place addition test passed." << std::endl;

  Srand2 = SrandRot*5.0;
  assert(Srand2.axis().approxEq(SrandRot.axis()));
  assert(fabs(Srand2.angle() - 5.0*SrandRot.angle()) < 1e-5);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Multiplication by value test passed." << std::endl;

  Srand2 = SrandRot;
  Srand2 *= 6.0;
  assert(Srand2.axis().approxEq(SrandRot.axis()));
  assert(fabs(Srand2.angle() - 6.0*SrandRot.angle()) < 1e-5);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place multiplication by value test passed." << std::endl;

  Srand2 = 3.0*SrandRot;
  assert(Srand2.axis().approxEq(SrandRot.axis()));
  assert(fabs(Srand2.angle() - 3.0*SrandRot.angle()) < 1e-5);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Commutative multiplication by value test passed." << std::endl;

  assert(Srand2 == Srand2);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Equality test passed." << std::endl;

  assert(Srand2 != SrandRot);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Inequality test passed." << std::endl;

  assert(screws::Rotationd().log() == screws::Skewd());
  assert(SrandRot.approxEq(Rrand.log(), 1e-10));
  assert(SrandRot.approxEq(Rrand.skew(), 1e-10));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Conversion from rotation to skew passed." << std::endl;
}

void testHomogeneousTransforms()
{
  int testIdx = 1;
  if (SHOW_PRINT_OUTS) std::cout << " == HOMOGENEOUS TRANSFORM == " << std::endl;

  screws::HomogeneousTransform<double> Heye;
  assert(Heye.translation() == screws::Translationd());
  assert(Heye.rotation() == screws::Rotationd());
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Identity constructor test passed." << std::endl;

  screws::Rotationd randRot(screws::Vector3d((double)rand()/RAND_MAX,
                                             (double)rand()/RAND_MAX,
                                             (double)rand()/RAND_MAX),
                            1.999*M_PI*(double)rand()/RAND_MAX);

  screws::Translationd randTrans(100*(double)rand()/RAND_MAX,
                                 100*(double)rand()/RAND_MAX,
                                 100*(double)rand()/RAND_MAX);
  screws::HomogeneousTransformd randH(randRot, randTrans);

  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      assert(randH(i, j) == randRot(i, j));
      assert(randH(j, 3) == randTrans(j));
    }
  }

  assert(randH.rotation() == randRot);
  assert(randH.translation() == randTrans);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") R-T constructor test passed." << std::endl;

  randH.setTranslation(randTrans);
  randH.setRotation(randRot);
  assert(randH.rotation() == randRot);
  assert(randH.translation() == randTrans);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") R-T setter/getter test passed." << std::endl;

  try
  {
    assert(Heye.inv() == Heye);
    screws::HomogeneousTransformd randHInv = randH.inv();
    assert(Heye.approxEq(randHInv*randH));
  }
  catch(screws::ScrewException s)
  {
    std::cout << s.what() << std::endl;
    exit(1);
  }
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Homogeneous transform inversion test passed." << std::endl;

  assert(randH.approxEq(randH*randH*randH.inv()));
  assert(Heye.approxEq(randH.inv()*randH*randH*randH.inv()));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Multiplication with homogeneous transform test passed." << std::endl;

  screws::HomogeneousTransformd randHold = randH;

  randH *= randH;
  randH *= randHold.inv();

  assert(randH.rotation().approxEq(randHold.rotation()));
  assert(randH.translation().approxEq(randHold.translation()));
  assert(randH.approxEq(randHold));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place multiplication with homogeneous transform test passed." << std::endl;

  assert(randTrans.approxEq(randH.inv()*(randH*randTrans)));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Multiplication with translation test passed." << std::endl;

  assert(randH == randH);
  assert(!(randH != randH));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Exact equality and inequality test passed." << std::endl;

}

void testRotations()
{
  if (SHOW_PRINT_OUTS) std::cout << " == ROTATION == " << std::endl;
  int testIdx = 1;

  screws::Rotation<double> Reye;
  screws::Rotation<double> Rx('x', M_PI/4);
  screws::Rotation<double> Ry('y', M_PI/4);
  screws::Rotation<double> Rz('z', M_PI/4);

  assert(Reye(0, 0) == 1.0 && Reye(0, 1) == 0.0 && Reye(0, 2) == 0.0 &&
         Reye(1, 0) == 0.0 && Reye(1, 1) == 1.0 && Reye(1, 2) == 0.0 &&
         Reye(2, 0) == 0.0 && Reye(2, 1) == 0.0 && Reye(2, 2) == 1.0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Default initialisation test passed." << std::endl;

  assert(Rx(0, 0) == 1.0 && Rx(0, 1) == 0.0 && Rx(0, 2) == 0.0 &&
         Rx(1, 0) == 0.0 && fabs(Rx(1, 1) - 0.707106781186548) < FLT_EPSILON && fabs(Rx(1, 2) + 0.707106781186548) < FLT_EPSILON &&
         Rx(2, 0) == 0.0 && fabs(Rx(2, 1) - 0.707106781186548) < FLT_EPSILON && fabs(Rx(2, 2) - 0.707106781186548) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Rx initialisation test passed." << std::endl;

  assert(fabs(Ry(0, 0) - 0.707106781186548) < FLT_EPSILON  && Ry(0, 1) == 0.0 && fabs(Ry(0, 2) - 0.707106781186548) < FLT_EPSILON &&
         Ry(1, 0) == 0.0                && Ry(1, 1) == 1.0 && Ry(1, 2) == 0.0 &&
         fabs(Ry(2, 0) + 0.707106781186548) < FLT_EPSILON && Ry(2, 1) == 0.0 && fabs(Ry(2, 2) - 0.707106781186548) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Ry initialisation test passed." << std::endl;

  assert(fabs(Rz(0, 0) - 0.707106781186548) < FLT_EPSILON && fabs(Rz(0, 1) + 0.707106781186548) < FLT_EPSILON && Rz(0, 2) == 0.0 &&
         fabs(Rz(1, 0) - 0.707106781186548) < FLT_EPSILON && fabs(Rz(1, 1) - 0.707106781186548) < FLT_EPSILON  && Rz(1, 2) == 0.0 &&
         Rz(2, 0) == 0.0               && Rz(2, 1) == 0.0                && Rz(2, 2) == 1.0);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Rz initialisation test passed." << std::endl;

  try
  {
    assert(Reye == Rx*Rx.inv());
    assert(Reye == Ry*Ry.inv());
    assert(Reye == Rz*Rz.inv());
    if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Invertion of basic rotations test passed." << std::endl;
  }
  catch (screws::ScrewException s)
  {
    if (SHOW_PRINT_OUTS) std::cout << "Exception raised: " << s.what() << std::endl;
    exit(1);
  }

  // Random rotations around x
  double angleX = 1.999*M_PI*((double)rand()/RAND_MAX);
  double angleY = 1.999*M_PI*((double)rand()/RAND_MAX);
  double angleZ = 1.999*M_PI*((double)rand()/RAND_MAX);
  screws::Rotation<double> Rxrand('x', angleX);
  screws::Rotation<double> Ryrand('y', angleY);
  screws::Rotation<double> Rzrand('z', angleZ);

  assert(Rxrand.isValid());
  assert(Ryrand.isValid());
  assert(Rzrand.isValid());

  // Inversion
  assert(Reye.approxEq(Rxrand*Rxrand.inv()));
  assert(Reye.approxEq(Ryrand*Ryrand.inv()));
  assert(Reye.approxEq(Rzrand*Rzrand.inv()));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Invertion of random rotations test passed." << std::endl;

  // RPY
  assert(fabs(Rxrand.rpy()(0) - angleX) < FLT_EPSILON && fabs(Rxrand.rpy()(1)) < FLT_EPSILON && fabs(Rxrand.rpy()(2)) < FLT_EPSILON);
  assert(fabs(Ryrand.rpy()(1) - angleY) < FLT_EPSILON && fabs(Ryrand.rpy()(0)) < FLT_EPSILON && fabs(Ryrand.rpy()(2)) < FLT_EPSILON);
  assert(fabs(Rzrand.rpy()(2) - angleZ) < FLT_EPSILON && fabs(Rzrand.rpy()(0)) < FLT_EPSILON && fabs(Rzrand.rpy()(1)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") RPY of basic axis test passed." << std::endl;

  // Axis
  screws::Vector3d randAxis(rand(), rand(), rand());
  randAxis = randAxis.normalised();
  double randAngle = (1.999*M_PI*rand())/RAND_MAX;
  screws::Rotationd Rrand(randAxis, randAngle);
  assert(Rrand.isValid());

  assert(randAxis.approxEq(Rrand.axis()));
  assert(fabs(randAngle - Rrand.angle()) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Axis and angle of random rotation test passed." << std::endl;

  screws::Rotationd RrandSquare = Rrand*Rrand;
  assert(RrandSquare.approxEq(screws::Rotation<double>(randAxis, 2*randAngle)));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Multiplication of random rotation test passed." << std::endl;

  RrandSquare *= Rrand;
  assert(RrandSquare.approxEq(screws::Rotation<double>(randAxis, 3*randAngle)));
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") In-place multiplication of random rotation test passed." << std::endl;

  screws::Rotation<double> rFromZ(screws::Vector3<double>(Rrand(0, 2), Rrand(1, 2), Rrand(2, 2)));
  assert(fabs(rFromZ(0, 2) - Rrand(0, 2)) < FLT_EPSILON &&
         fabs(rFromZ(1, 2) - Rrand(1, 2)) < FLT_EPSILON &&
         fabs(rFromZ(2, 2) - Rrand(2, 2)) < FLT_EPSILON);
  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") Creation of rotation from z-axis passed." << std::endl;

  double randomRoll = 1.999*M_PI*(double)rand()/RAND_MAX;
  double randomPitch = 1.999*M_PI*(double)rand()/RAND_MAX;
  double randomYaw = 1.999*M_PI*(double)rand()/RAND_MAX;

  Rrand = screws::Rotationd('z', randomYaw)*screws::Rotationd('y', randomPitch)*screws::Rotationd('x', randomRoll);
  screws::Rotationd Rrand2 = screws::Rotationd('z', Rrand.rpy()(2))*screws::Rotationd('y', Rrand.rpy()(1))*screws::Rotationd('x', Rrand.rpy()(0));
  assert(Rrand.approxEq(Rrand2));

  if (SHOW_PRINT_OUTS) std::cout << testIdx++ << ") RPY and creation from RPY test passed." << std::endl;
}

int main(void)
{
  srand(time(NULL));

  const int maxIter = 5000000;
  if (TEST_VECTOR6)
  {
    for(int i = 1; i <= maxIter; ++i)
    {
      if (i % 10000 == 0)
        std::cout << "Vector6 iteration " << i << " of " << maxIter << std::endl;
      testVector6();
    }
    std::cout << "\n\n" << std::endl;
  }

  if (TEST_TRANSLATIONS)
  {
    for(int i = 1; i <= maxIter; ++i)
    {
      if (i % 10000 == 0)
        std::cout << "Translation iteration " << i << " of " << maxIter << std::endl;
      testTranslations();
    }
    std::cout << "\n\n" << std::endl;
  }

  if (TEST_ROTATIONS)
  {
    for(int i = 1; i <= maxIter; ++i)
    {
      if (i % 10000 == 0)
        std::cout << "Rotation iteration " << i << " of " << maxIter << std::endl;
      testRotations();
    }
    std::cout << "\n\n" << std::endl;
  }

  if (TEST_HOMOGENEOUS_TRANSFORMS)
  {
    for(int i = 1; i <= maxIter; ++i)
    {
      if (i % 10000 == 0)
        std::cout << "HomogeneousTransform iteration " << i << " of " << maxIter << std::endl;
      testHomogeneousTransforms();
    }
    std::cout << "\n\n" << std::endl;
  }

  if (TEST_SKEWS)
  {
    for(int i = 1; i <= maxIter; ++i)
    {
      if (i % 10000 == 0)
        std::cout << "Skew iteration " << i << " of " << maxIter << std::endl;
      testSkews();
    }
    std::cout << "\n\n" << std::endl;
  }

  if (TEST_TWISTS)
  {
    for(int i = 1; i <= maxIter; ++i)
    {
      if (i % 10000 == 0)
        std::cout << "Twist iteration " << i << " of " << maxIter << std::endl;
      testTwists();
    }
  }
  return 0;
}
