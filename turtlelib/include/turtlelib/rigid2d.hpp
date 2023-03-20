#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cstdlib> // it is the c++ standard library headerfile
#include<vector>


namespace turtlelib
{
   /// \brief PI.  Not in C++ standard until C++20.
   constexpr double PI=3.14159265358979323846;


   /// \brief approximately compare two floating-point numbers using
   ///        an absolute comparison
   /// \param d1 - a number to compare
   /// \param d2 - a second number to compare
   /// \param epsilon - absolute threshold required for equality
   /// \return true if abs(d1 - d2) < epsilon
   /// NOTE: implement this in the header file
   /// constexpr means that the function can be computed at compile time
   /// if given a compile-time constant as input
   constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
   {
        return (std::abs(d1 - d2) < epsilon);
   }


   /// \brief convert degrees to radians
   /// \param deg - angle in degrees
   /// \returns radians
   constexpr double deg2rad(double deg)
   {
       return deg * PI / 180.0;
   }


   /// \brief convert radians to degrees
   /// \param rad - angle in radians
   /// \returns the angle in degrees
   constexpr double rad2deg(double rad)
   {
       return rad * 180 / PI;
   }


   /// static_assertions test compile time assumptions.
   /// You should write at least one more test for each function
   /// You should also purposely (and temporarily) make one of these tests fail
   /// just to see what happens
   static_assert(almost_equal(0, 0), "is_zero failed");


   static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");


   static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");


   static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
   /// change value of epsilon to temporarily fail assertion
   static_assert(almost_equal(-10.5439, -11.0, 0.5), "Negative number comparison failed");


   static_assert(almost_equal(90.0, rad2deg(deg2rad(90.0))), "Comparison failed");


   static_assert(almost_equal(deg2rad(90.0), 1.570796, 0.005), "rad2deg failed");


   /// \brief A 2-Dimensional Vector
   struct Vector2D
   {
       /// \brief the x coordinate
       double x = 0.0;


       /// \brief the y coordinate
       double y = 0.0;


       /// \brief add the vector to given vector
       /// \param rhs - the first vector to add with
       /// \return a reference to the newly summed vector
       Vector2D & operator+=(const Vector2D & rhs);


       /// \brief subtract a vector from given vector
       /// \param rhs - the vector to subtract with
       /// \return a reference to the newly subtracted vector
       Vector2D & operator-=(const Vector2D & rhs);


       /// \brief Multiply the scalar to given vector
       /// \param rhs - the vector to multiply with
       /// \return a reference to the newly multiplied vector
       Vector2D & operator*=(const double rhs);


   };


   /// \brief calculate the addition the 2 vectors
   /// \param vec1 - the first vector to add
   /// \param vec2 - the second vector to add
   /// \return the summation vector
   Vector2D operator+(Vector2D vec1, Vector2D vec2);


   /// \brief calculate the subtraction the 2 vectors
   /// \param vec1 - the first vector to subtract from
   /// \param vec2 - the second vector
   /// \return the difference vector
   Vector2D operator-(Vector2D vec1, Vector2D vec2);


   /// \brief calculate the product of vector with a scalar on the right
   /// \param vec1 - the first vector to multiply to
   /// \param scal - the scalar to multiply with
   /// \return the resulting vector
   Vector2D operator*(Vector2D vec1, double scal);


   /// \brief calculate the product of vector with a scalar on the left
   /// \param vec1 - the first vector to multiply to
   /// \param scal - the scalar to multiply with
   /// \return the resulting vector
   Vector2D operator*(double scal, Vector2D vec1);


   /// \brief The dot product of two vectors
   /// \param vec1 - the first vector
   /// \param vec2 - the second vector
   /// \return the dot product
   double dot(Vector2D vec1, Vector2D vec2);


   /// \brief computer the magnitude of the vector
   /// \param vec - the vector whose magnitude is calculated
   /// \return the magnitude
   double magnitude(Vector2D vec);


   /// \brief compute the angle between two vectors
   /// \param vec1 - the first vector
   /// \param vec2 - the second vector
   /// \return the calculated angle
   double angle(Vector2D vec1, Vector2D vec2);


   /// \brief A 2-Dimensional Twist
   struct Twist2D
   {
       /// \brief the angular velocity
       double w = 0.0;


       /// \brief the x velocity
       double x = 0.0;


       /// \brief the y velocity
       double y = 0.0;
   };




   /// \brief output a 2 dimensional twist as [angularcomponent xcomponent ycomponent]
   /// os - stream to output to
   /// t - the twist to print
   std::ostream & operator<<(std::ostream & os, const Twist2D & t);


   /// \brief input a 2 dimensional twist
   ///   You should be able to read twistss entered as follows:
   ///   [w x y] or w x y
   /// \param is - stream from which to read
   /// \param t [out] - output twist
   std::istream & operator>>(std::istream & is, Twist2D & t);


   /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
   /// os - stream to output to
   /// v - the vector to print
   std::ostream & operator<<(std::ostream & os, const Vector2D & v);


   /// \brief input a 2 dimensional vector
   ///   You should be able to read vectors entered as follows:
   ///   [x y] or x y
   /// \param is - stream from which to read
   /// \param v [out] - output vector
   ///
   /// The way input works is (more or less): what the user types is stored in a buffer until the user types
   /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
   /// Typically, each character is examined and then removed from the buffer automatically.
   /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
   /// an error flag is set on the stream object (e.g., std::cin) and processing stops.
   ///
   /// We have lower level control however.
   /// std::peek() looks at the next unprocessed character in the buffer without removing it
   ///     https://en.cppreference.com/w/cpp/io/basic_istream/peek
   /// std::get() removes the next unprocessed character from the buffer.
   ///     https://en.cppreference.com/w/cpp/io/basic_istream/get
   /// When you call std::peek() it will wait for there to be at least one character in the buffer (e.g., the user types a character)
   /// HINT: this function can be written in under 20 lines and uses only std::peek(), std::get(), istream::operator>>() and a little logic
   std::istream & operator>>(std::istream & is, Vector2D & v);


   /// \brief a rigid body transformation in 2 dimensions
   class Transform2D
   {
   private:
       Vector2D vec;
       double theta;
   public:
       /// \brief Create an identity transformation
       Transform2D();


       /// \brief create a transformation that is a pure translation
       /// \param trans - the vector by which to translate
       explicit Transform2D(Vector2D trans);


       /// \brief create a pure rotation
       /// \param radians - angle of the rotation, in radians
       explicit Transform2D(double radians);


       /// \brief Create a transformation with a translational and rotational
       /// component
       /// \param trans - the translation
       /// \param radians - the rotation, in radians
       Transform2D(Vector2D trans, double radians);


       /// \brief apply a transformation to a Vector2D
       /// \param v - the vector to transform
       /// \return a vector in the new coordinate system
       Vector2D operator()(Vector2D v) const;


       /// \brief invert the transformation
       /// \return the inverse transformation.
       Transform2D inv() const;


       /// \brief compose this transform with another and store the result
       /// in this object
       /// \param rhs - the first transform to apply
       /// \return a reference to the newly transformed operator
       Transform2D & operator*=(const Transform2D & rhs);


       /// \brief the translational component of the transform
       /// \return the x,y translation
       Vector2D translation() const;


       /// \brief get the angular displacement of the transform
       /// \return the angular displacement, in radians
       double rotation() const;


       /// \brief convert a twist to a different reference frame using the adjoint
       /// \param v - the twist in the original reference frame
       /// \return the twist in a different reference frame
       Twist2D twistconversion(Twist2D v) const;


       /// \brief \see operator<<(...) (declared outside this class)
       /// for a description
       friend std::ostream & operator<<(std::ostream & os, const Transform2D & tf);
   };


   /// \brief should print a human readable version of the transform:
   /// An example output:
   /// deg: 90 x: 3 y: 5
   /// \param os - an output stream
   /// \param tf - the transform to print
   std::ostream & operator<<(std::ostream & os, const Transform2D & tf);


   /// \brief Read a transformation from stdin
   /// Should be able to read input either as output by operator<< or
   /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
   /// For example:
   /// 90 2 3
   std::istream & operator>>(std::istream & is, Transform2D & tf);


   /// \brief multiply two transforms together, returning their composition
   /// \param lhs - the left hand operand
   /// \param rhs - the right hand operand
   /// \return the composition of the two transforms
   /// HINT: This function should be implemented in terms of *=
   Transform2D operator*(Transform2D lhs, const Transform2D & rhs);


   /// \brief normalize a Vector2D(i.e., find the unit vector in the direction of a given Vector2D)
   /// \param vector - the vector that will be normalized
   /// \return the normalized vector
   Vector2D normalize(Vector2D vector);


   /// \brief normalize an angle (turn any angle to it equivalent angle in the interval pi to -pi)
   /// \param rad - the anglethat will be normalized
   /// \return the normalized angle
   double normalize_angle(double rad);


   /// \brief compute the transformation corresponding to a rigid body following a constant twist 
   /// (in its original body frame) for one time-unit
   /// \param Vb - the a constant twist (
   /// \return The resulting transformation matrix
   Transform2D integrate_twist(Twist2D Vb);
}


#endif