#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle_fitting.hpp"
#include <sstream>


TEST_CASE( "Operator () for Vector2D", "[transform]" ) { // Yin, Hang
   double my_x = 2;
   double my_y = 3;
   double my_ang = turtlelib::PI;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Vector2D v = {2,2};
   turtlelib::Vector2D result = Ttest(v);
   REQUIRE( turtlelib::almost_equal(result.x, 0.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(result.y, 1.0, 1.0e-5) );
}

TEST_CASE( "Inverse", "[transform]" ) { // Hughes, Katie
   float my_x = 0.;
   float my_y = 1.;
   float my_ang = turtlelib::PI/2;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Transform2D Ttest_inv = Ttest.inv();
   REQUIRE( (Ttest.inv()).rotation() == -my_ang);
   REQUIRE( turtlelib::almost_equal(Ttest_inv.translation().x, -1.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(Ttest_inv.translation().y,  0.0, 1.0e-5) );
}

TEST_CASE("operator *=", "[transform]"){    //Megan, Sindelar
   turtlelib::Vector2D trans_ab = {1,2};
   double rotate_ab = 0;
   turtlelib::Transform2D T_ab_1 = {trans_ab, rotate_ab};      //T_ab's are all the same,
   turtlelib::Transform2D T_ab_2 = {trans_ab, rotate_ab};      //but, need different vars
   turtlelib::Transform2D T_ab_3 = {trans_ab, rotate_ab};      //b/c getting overwritten otherwise
   turtlelib::Vector2D trans_bc = {3,4};
   double rotate_bc = turtlelib::PI/2;
   turtlelib::Transform2D T_bc = {trans_bc, rotate_bc};
 
   REQUIRE(turtlelib::almost_equal((T_ab_1*=T_bc).translation().x, 4.0));
   REQUIRE(turtlelib::almost_equal((T_ab_2*=T_bc).translation().y, 6.0));
   REQUIRE(turtlelib::almost_equal((T_ab_3*=T_bc).rotation(), (turtlelib::PI/2)));
}

TEST_CASE("Translation()","[transform]"){ // Marno, Nel
   double test_x = 4.20;
   double test_y = 6.9;
   turtlelib::Transform2D T_test{{test_x,test_y}};
   REQUIRE(T_test.translation().x == test_x);
   REQUIRE(T_test.translation().y == test_y);
}

TEST_CASE("Rotation()","[transform]"){ //Ghosh, Ritika
    double test_ang = turtlelib::PI/4; //It is assumed this function takes radians
    turtlelib::Vector2D test_vec = {3.5, 2.5};
    turtlelib::Transform2D Ttest1, Ttest2(test_ang), Ttest3(test_vec), Ttest4(test_vec, test_ang);
    REQUIRE(Ttest1.rotation() == 0.0);
    REQUIRE(Ttest2.rotation() == test_ang);
    REQUIRE(Ttest3.rotation() == 0.0);
    REQUIRE(Ttest4.rotation() == test_ang);
}
TEST_CASE("twistconversion()","[transform]"){ //Ghosh, Ritika
    turtlelib::Transform2D Tab={turtlelib::Vector2D{0,1}, turtlelib::PI/2};
    turtlelib::Transform2D Tbc={turtlelib::Vector2D{1,0}, turtlelib::PI/2};
    turtlelib::Twist2D Vb = {1, 1, 1}, Va = Tab.twistconversion(Vb), 
                        Vc = (Tbc.inv()).twistconversion(Vb);
    REQUIRE_THAT(Va.w, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(Va.x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(Va.y, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(Vc.w, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(Vc.x, Catch::Matchers::WithinAbs(2.0, 1.0e-5));
    REQUIRE_THAT(Vc.y, Catch::Matchers::WithinAbs(-1.0, 1.0e-5));
}

TEST_CASE("operator <<","[transform]"){ //Ghosh, Ritika
    turtlelib::Transform2D Ttest={turtlelib::Vector2D{0,1}, turtlelib::PI/2};
    std::ostringstream os;
    os << Ttest;
    REQUIRE(os.str()=="deg: 90 x: 0 y: 1");
}

TEST_CASE("operator >>","[transform]"){ //Ghosh, Ritika
    turtlelib::Transform2D Ttest1, Ttest2;
    std::istringstream is1("deg: 90 x: 0 y: 1"), is2("45 1 0");
    is1 >> Ttest1;
    is2 >> Ttest2;
    REQUIRE_THAT(Ttest1.rotation(), Catch::Matchers::WithinAbs(turtlelib::PI/2, 1.0e-5));
    REQUIRE_THAT(Ttest1.translation().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(Ttest1.translation().y, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(Ttest2.rotation(), Catch::Matchers::WithinAbs(turtlelib::PI/4, 1.0e-5));
    REQUIRE_THAT(Ttest2.translation().x, Catch::Matchers::WithinAbs(1.0, 1.0e-5));
    REQUIRE_THAT(Ttest2.translation().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
}

TEST_CASE("normalize_angle","[rigid2d]"){ //Ghosh, Ritika
   double pi = turtlelib::PI;
   double ang1=pi, ang2=-pi, ang3=0.0, ang4 = -pi/4, ang5 = 3*pi/2, ang6= -5*pi/2, ang7 = -4*pi/3, ang8= -7*pi/4;
   REQUIRE_THAT(turtlelib::normalize_angle(ang1), Catch::Matchers::WithinAbs(pi, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(ang2), Catch::Matchers::WithinAbs(pi, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(ang3), Catch::Matchers::WithinAbs(0.0, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(ang4), Catch::Matchers::WithinAbs(-pi/4, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(ang5), Catch::Matchers::WithinAbs(-pi/2, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(ang6), Catch::Matchers::WithinAbs(-pi/2, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(-2*pi), Catch::Matchers::WithinAbs(0.0, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(-5*pi), Catch::Matchers::WithinAbs(pi, 1.0e-5));

   REQUIRE_THAT(turtlelib::normalize_angle(ang7), Catch::Matchers::WithinAbs(2*pi/3, 1.0e-5));
   REQUIRE_THAT(turtlelib::normalize_angle(ang8), Catch::Matchers::WithinAbs(pi/4, 1.0e-5));


}

TEST_CASE("integrate_twist", "[rigid2D]"){   // Ghosh, Ritika
   turtlelib::Twist2D t1={0.785, 0.0, 0.0}, t2={0.0, 1.56, -3.22}, t3={0.785, 1.56, -3.22};
   turtlelib::Transform2D Ttest1 = turtlelib::integrate_twist(t1);
   turtlelib::Transform2D Ttest2 = turtlelib::integrate_twist(t2);
   turtlelib::Transform2D Ttest3 = turtlelib::integrate_twist(t3);
   turtlelib::Vector2D trans1 = Ttest1.translation();
   turtlelib::Vector2D trans2 = Ttest2.translation();
   turtlelib::Vector2D trans3 = Ttest3.translation();
   double ang1 = Ttest1.rotation(), ang2 = Ttest2.rotation(), ang3 = Ttest3.rotation();
   REQUIRE_THAT(ang1, Catch::Matchers::WithinAbs(0.785, 1e-5));
   REQUIRE_THAT(trans1.x, Catch::Matchers::WithinAbs(0.0, 1e-5));
   REQUIRE_THAT(trans1.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
   REQUIRE_THAT(ang2, Catch::Matchers::WithinAbs(0.0, 1e-5));
   REQUIRE_THAT(trans2.x, Catch::Matchers::WithinAbs(1.56, 1e-5));
   REQUIRE_THAT(trans2.y, Catch::Matchers::WithinAbs(-3.22, 1e-5));
   REQUIRE_THAT(ang3, Catch::Matchers::WithinAbs(0.785, 1e-5));
   REQUIRE_THAT(trans3.x, Catch::Matchers::WithinAbs(2.60491, 1e-5));
   REQUIRE_THAT(trans3.y, Catch::Matchers::WithinAbs(-2.31783, 1e-5));
  
}


TEST_CASE("Forward Kinematics", "[DiffDrive]") {   // Ghosh, Ritika
    turtlelib::Wheel_ang phi1, phi2, phi3;
    phi1.left = 0.2;
    phi1.right = 0.2;
    turtlelib::DiffDrive Ttest1, Ttest2, Ttest3, Ttest4({turtlelib::PI/4, 0.5, -1.0}, {turtlelib::PI/12, turtlelib::PI/12}, 1.0, 4.0);
    Ttest1.FwdK(phi1);
    //for forward motion
    REQUIRE_THAT(Ttest1.get_config().theta, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(Ttest1.get_config().x, Catch::Matchers::WithinAbs(0.2, 1.0e-5));
    REQUIRE_THAT(Ttest1.get_config().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));

    phi2.left = 0.2;
    phi2.right = -0.2;
    Ttest2.FwdK(phi2);
    //for pure rotation
    REQUIRE_THAT(Ttest2.get_config().theta, Catch::Matchers::WithinAbs(-0.1, 1.0e-5));
    REQUIRE_THAT(Ttest2.get_config().x, Catch::Matchers::WithinAbs(0.0, 1.0e-5));
    REQUIRE_THAT(Ttest2.get_config().y, Catch::Matchers::WithinAbs(0.0, 1.0e-5));

    phi3.left = -1.0;
    phi3.right = 0.5;
    Ttest3.FwdK(phi3);
    //for robot moving in an arc
    REQUIRE_THAT(Ttest3.get_config().theta, Catch::Matchers::WithinAbs(0.375, 1.0e-5));
    REQUIRE_THAT(Ttest3.get_config().x, Catch::Matchers::WithinAbs(-0.244182, 1.0e-5));
    REQUIRE_THAT(Ttest3.get_config().y, Catch::Matchers::WithinAbs(-0.0463283, 1.0e-5));

    Ttest4.FwdK(phi1);
    //for robot moving moving straight but with different intial condition
    REQUIRE_THAT(Ttest4.get_config().theta, Catch::Matchers::WithinAbs(0.785398, 1.0e-5));
    REQUIRE_THAT(Ttest4.get_config().x, Catch::Matchers::WithinAbs(0.64142, 1.0e-5));
    REQUIRE_THAT(Ttest4.get_config().y, Catch::Matchers::WithinAbs(-0.85857, 1.0e-5));

}

TEST_CASE("Inverse Kinematics", "[Diffdrive]") {   // Ghosh, Ritika
    turtlelib::Twist2D Vb1 {0.0, 0.25, 0.0}, Vb2{0.54, 0.0, 0.0}, Vb3{0.785, 1.20, 0.0}, Vb4{0.25, 1.5, 1.5};
    turtlelib::DiffDrive Ttest1, Ttest2, Ttest3, Ttest4;
    turtlelib::Wheel_ang p1 = Ttest1.InvK(Vb1), p2 = Ttest1.InvK(Vb2), p3 = Ttest1.InvK(Vb3);

    //for forward motion
    REQUIRE_THAT(p1.right, Catch::Matchers::WithinAbs(0.25, 1.0e-5));
    REQUIRE_THAT(p1.left, Catch::Matchers::WithinAbs(0.25, 1.0e-5));

    //for pure rotation
    REQUIRE_THAT(p2.right, Catch::Matchers::WithinAbs(1.08, 1.0e-5));
    REQUIRE_THAT(p2.left, Catch::Matchers::WithinAbs(-1.08, 1.0e-5));

    //for robot moving in an arc
    REQUIRE_THAT(p3.right, Catch::Matchers::WithinAbs(2.77, 1.0e-5));
    REQUIRE_THAT(p3.left, Catch::Matchers::WithinAbs(-0.37, 1.0e-5));

    //Impossible case
    REQUIRE_THROWS_AS(Ttest4.InvK(Vb4), std::logic_error);
}

TEST_CASE("Fit Circle", "[CircleFit]") {   // Ghosh, Ritika
    std::vector<turtlelib::Coord> test1{{1, 7}, {2, 6}, {5, 8}, {7, 7}, {9, 5}, {3, 7}};
    std::vector<turtlelib::Coord> test2{{-1, 0}, {-0.3, -0.06}, {0.3, 0.1}, {1, 0}};

    REQUIRE_THAT(turtlelib::fit_circle(test1).at(0), Catch::Matchers::WithinAbs(4.615482, 1.0e-4));
    REQUIRE_THAT(turtlelib::fit_circle(test1).at(1), Catch::Matchers::WithinAbs(2.807354, 1.0e-4));
    REQUIRE_THAT(turtlelib::fit_circle(test1).at(2), Catch::Matchers::WithinAbs(4.8275, 1.0e-4));

    REQUIRE_THAT(turtlelib::fit_circle(test2).at(0), Catch::Matchers::WithinAbs(0.4908357, 1.0e-4));
    REQUIRE_THAT(turtlelib::fit_circle(test2).at(1), Catch::Matchers::WithinAbs(-22.15212, 1.0e-4));
    REQUIRE_THAT(turtlelib::fit_circle(test2).at(2), Catch::Matchers::WithinAbs(22.17979, 1.0e-4));
}
