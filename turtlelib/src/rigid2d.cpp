#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <cstdio>
#include <cmath>

turtlelib::Vector2D &turtlelib::Vector2D::operator+=(const turtlelib::Vector2D &rhs){
    x += rhs.x;
    y += rhs.y;

    return *this;
}

turtlelib::Vector2D &turtlelib::Vector2D::operator-=(const turtlelib::Vector2D &rhs){
    x -= rhs.x;
    y -= rhs.y;

    return *this;
}

turtlelib::Vector2D &turtlelib::Vector2D::operator*=(const double rhs){
    x *= rhs;
    y *= rhs;

    return *this;
}

turtlelib::Vector2D turtlelib::operator+(turtlelib::Vector2D vec1, turtlelib::Vector2D vec2){
    vec1 += vec2;

    return vec1;
}

turtlelib::Vector2D turtlelib::operator-(turtlelib::Vector2D vec1, turtlelib::Vector2D vec2){
    vec1 -= vec2;

    return vec1;
}

turtlelib::Vector2D turtlelib::operator*(turtlelib::Vector2D vec1, double scal){
    vec1 *= scal;

    return vec1;
}

turtlelib::Vector2D turtlelib::operator*(double scal, turtlelib::Vector2D vec1){
    vec1 *= scal;

    return vec1;
}

double turtlelib::dot(turtlelib::Vector2D vec1, turtlelib::Vector2D vec2){
    double dot=0.0;
    dot = vec1.x*vec2.x + vec1.y*vec2.y;

    return dot;
}

double turtlelib::magnitude(turtlelib::Vector2D vec){
    double mag = sqrt((vec.x*vec.x) + (vec.y*vec.y));

    return mag;
}

double turtlelib::angle(turtlelib::Vector2D vec1, turtlelib::Vector2D vec2){
    double a = atan2(vec1.x*vec2.y-vec1.y*vec2.x,vec1.x*vec2.x+vec1.y*vec2.y);

    return a;
}

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Vector2D &v){
    os << "[" << v.x << " " << v.y << "]";
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Vector2D &v){
    if(is.peek() == '['){
    is.get();
    }
    is >> v.x >> v.y;
    is.clear();
    is.ignore(100, '\n');

    return is;
}

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Twist2D &t){
    os << "[" << t.w << " " << t.x << " " << t.y << "]";
    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Twist2D &t){
    if(is.peek() == '['){
        is.get();
    }
    is >> t.w >> t.x >> t.y;
    is.clear();
    is.ignore(100, '\n');

    return is;
}

//contructor definition
turtlelib::Transform2D::Transform2D() : vec{0.0, 0.0}, theta(0.0){}

turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans) : vec(trans), theta(0){}

turtlelib::Transform2D::Transform2D(double radians) : vec{0.0, 0.0}, theta(radians){}

turtlelib::Transform2D::Transform2D(turtlelib::Vector2D trans, double radians) :
vec(trans), theta(radians){}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const{

    return {cos(theta)* v.x - sin(theta)*v.y + vec.x, sin(theta)*v.x + cos(theta)*v.y + vec.y};
}

turtlelib::Transform2D turtlelib::Transform2D::inv() const{

    return {{-(vec.x * cos(theta) + vec.y*sin(theta)),(vec.x*sin(theta) - vec.y*cos(theta))}, -theta};
}

turtlelib::Transform2D &turtlelib::Transform2D::operator*=(const turtlelib::Transform2D &rhs){
    const auto t = theta, x = vec.x, y= vec.y;
    this->theta = t + rhs.theta;
    // this->theta = fmod(t + rhs.theta, 2*PI); //considering angle wrapping
    this->vec.x = rhs.vec.x*cos(t) - rhs.vec.y*sin(t) + x;
    this->vec.y = rhs.vec.x*sin(t) + rhs.vec.y*cos(t) + y;

    return *this;
}


turtlelib::Vector2D turtlelib::Transform2D::translation() const{

    return vec;
}

double turtlelib::Transform2D::rotation() const{
    return theta; //assuming the theta in transform2D is in radians
}

turtlelib::Twist2D turtlelib::Transform2D::twistconversion(turtlelib::Twist2D v) const{

    return {v.w, v.w*vec.y + v.x*cos(theta) - v.y*sin(theta), -v.w*vec.x + v.x*sin(theta) + v.y*cos(theta)};
}

std::ostream &turtlelib::operator<<(std::ostream &os, const turtlelib::Transform2D &tf){
    os << "deg: " << turtlelib::rad2deg(tf.theta) << " x: " << tf.vec.x << " y: " << tf.vec.y;

    return os;
}

std::istream &turtlelib::operator>>(std::istream &is, turtlelib::Transform2D &tf){
    double t;
    Vector2D v;
    if(is.peek() == 'd'){
        char v1[]="deg:", v2[]="x:", v3[]="y:";
        is >> v1 >> t >> v2 >> v.x >> v3 >> v.y;
        is.clear();
        is.ignore(50, '\n');
    }
    else{
        is >> t >> v.x >> v.y;
        is.clear();
        is.ignore(50, '\n');
    }

    turtlelib::Transform2D temp(v, turtlelib::deg2rad(t));
    tf = temp;

    return is;
}

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D &rhs){
    lhs *= rhs;

    return lhs;
}

turtlelib::Vector2D turtlelib::normalize(turtlelib::Vector2D vector){
    const auto mag = sqrt((vector.x*vector.x) + (vector.y*vector.y));

    return {vector.x/mag, vector.y/mag};
}

double turtlelib::normalize_angle(double rad){
    double n_ang=fmod(rad, 2*PI);
    // if(rad > PI){
    //     n_ang = fmod(rad, PI);
    // }
    // else if(rad <= -PI){
    //     n_ang = fmod(rad, -PI);
    //     if (turtlelib::almost_equal(n_ang, 0.0, 1.0e-5)){
    //         n_ang = -fmod(rad, 2*PI);
    //     }
    // }
    // else{
    //     n_ang = rad;
    // }
    // if (n_ang >= 2*PI){
    //     n_ang = fmod(n_ang, 2*PI);
    // } else if(n_ang <= -2*PI){
    //     n_ang = -fmod(n_ang, 2*PI);
    // }
    if (n_ang > PI){
        n_ang = -PI + fmod(n_ang, PI);
    }else if (n_ang <= -PI){
        n_ang = PI + fmod(n_ang, PI);
    }

    return n_ang;
}

turtlelib::Transform2D turtlelib::integrate_twist(turtlelib::Twist2D Vb){
    turtlelib::Transform2D Tbb_dash;
    if (Vb.w ==0){
        Tbb_dash={turtlelib::Vector2D {Vb.x, Vb.y}, 0.0};
    }
    else{
        double cor_x = Vb.y/Vb.w;
        double cor_y = -Vb.x/Vb.w;
        turtlelib::Transform2D Tsb={turtlelib::Vector2D {cor_x, cor_y}, 0.0};
        turtlelib::Transform2D Tss_dash={turtlelib::Vector2D {0.0, 0.0}, Vb.w};
        Tbb_dash = Tsb.inv()*Tss_dash*Tsb;
    }

    return Tbb_dash;
}
