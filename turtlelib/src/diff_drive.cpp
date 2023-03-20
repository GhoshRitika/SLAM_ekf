#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <iostream>

// turtlelib::DiffDrive::DiffDrive(turtlelib::Config c, turtlelib::Wheel_ang w, double rad, double track):
//     q(c), phi(w), r(rad), D(track/2){}

turtlelib::DiffDrive::DiffDrive()://q{0.0, 0.0, 0.0}, phi{0.0, 0.0}, r(1.0), D(2.0){} 
DiffDrive({0.0, 0.0, 0.0}, {0.0, 0.0}, 1.0, 4.0){}

turtlelib::DiffDrive::DiffDrive(double rad, double track):
    DiffDrive({0.0, 0.0, 0.0}, {0.0, 0.0}, rad, track/2.0){}
    //q{0.0, 0.0, 0.0}, phi{0.0, 0.0}, r(rad), D(track/2){}


turtlelib::DiffDrive::DiffDrive(turtlelib::Config c, turtlelib::Wheel_ang w, double rad, double track):
    q(c), phi(w), r(rad), D(track/2){}


turtlelib::Config turtlelib::DiffDrive::get_config() const{
    return q;
}
turtlelib::Wheel_ang turtlelib::DiffDrive::get_phi() const{
    return phi;
}

void turtlelib::DiffDrive::set_phi(turtlelib::Wheel_ang phi_new){
    phi.right = phi_new.right;
    phi.left = phi_new.left;
}

void turtlelib::DiffDrive::set_config(turtlelib::Config config){
    q.x = config.x;
    q.y = config.y;
    q.theta = config.theta;
}

turtlelib::Twist2D turtlelib::DiffDrive::FwdK(turtlelib::Wheel_ang phi_new){
    // Calculating th change in wheel angles
    // const auto dphi_r = phi_new.right - phi.right, dphi_l = phi_new.left - phi.left;
    const auto dphi_r = phi_new.right , dphi_l = phi_new.left;
    // calculating the twist using the relation u=H*Vb
    const auto Vb_ang = r*0.5*(dphi_r- dphi_l) / D, Vb_x = r*0.5*(dphi_l+dphi_r);
    // Getting the body transformation by integrated the calculated body twist
    Transform2D Tbb_dash = integrate_twist({Vb_ang, Vb_x, 0.0});
    const auto theta_qb = Tbb_dash.rotation();
    Vector2D qb = Tbb_dash.translation();
    //Using the transformation to get the change in configuation
    const auto dq_x = qb.x*cos(q.theta) - qb.y*sin(q.theta);
    const auto dq_y = qb.x*sin(q.theta) + qb.y*cos(q.theta);
    //Adding the change in configuration to the previous configuration
    q.theta = q.theta + theta_qb;
    q.x = q.x + dq_x;
    q.y = q.y + dq_y;
    phi.right += phi_new.right;
    phi.left += phi_new.left;
    return {Vb_ang, Vb_x, 0.0};
}


turtlelib::Wheel_ang turtlelib::DiffDrive::InvK(turtlelib::Twist2D Vb){
    //Since the robot should not slip, the Y component of the twist should be 0
    if (Vb.y != 0.0){
        throw std::logic_error("Slipping not allowed");
    }

    return {((D*Vb.w + Vb.x)/ r), ((-D*Vb.w + Vb.x)/ r)};
}

