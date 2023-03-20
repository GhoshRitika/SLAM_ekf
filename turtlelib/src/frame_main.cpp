#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iostream>


int main(void){
    turtlelib::Transform2D Tab, Tbc;
    turtlelib::Vector2D vb;
    turtlelib::Twist2D Vb;
    std::cout << "Enter transform T_{a,b}: \n";
    std::cin >> Tab;


    std::cout << "Enter transform T_{b,c}: \n";
    std::cin >> Tbc;

    std::cout << "T_{a,b}:" << Tab;
    std::cout << "\nT_{b,a}:" << Tab.inv();
    std::cout << "\nT_{b,c}:" << Tbc;
    std::cout << "\nT_{c,b}:" << Tbc.inv();
    std::cout << "\nT_{a,c}:" << Tab*Tbc;
    std::cout << "\nT_{c,a}:" << (Tab*Tbc).inv();

    std::cout << "\nEnter a vector v_b: \n";
    std::cin >> vb;

    turtlelib::Vector2D vbhat = normalize(vb);
    turtlelib::Vector2D va = Tab(vb);
    turtlelib::Vector2D vc = (Tbc.inv())(vb);

    std::cout << "v_bhat: " << vbhat;
    std::cout << "\nv_a: " << va;
    std::cout << "\nv_b: " << vb;
    std::cout << "\nv_c: " << vc;

    std::cout << "\nEnter twist V_b: \n";
    std::cin >> Vb;
    
    turtlelib::Twist2D Va = Tab.twistconversion(Vb);
    turtlelib::Twist2D Vc = (Tbc.inv()).twistconversion(Vb);

    std::cout << "V_a: " << Va;
    std::cout << "\nV_b: " << Vb;
    std::cout << "\nV_c: " << Vc << std::endl;

    return 0;
}