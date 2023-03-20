#include "turtlelib/circle_fitting.hpp"
#include <cmath>
#include <algorithm>
#include <armadillo>

std::vector<double> turtlelib::fit_circle(std::vector<Coord> &cluster_real) {
    //find the mean of the x and y coordinates
    turtlelib::Coord center;
    double n = cluster_real.size();
    for(int i=0; i< n; i++){
        Coord p = cluster_real.at(i);
        center.x += p.x;
        center.y += p.y;
    }
    center.x = center.x /n;
    center.y = center.y /n;
    std::vector<turtlelib::Coord> cluster = cluster_real;
    // Shift the coordinates so that the centroid is at the origin
    for(int i=0; i< n; i++){
        cluster.at(i).x -= center.x;
        cluster.at(i).y -= center.y;
    }
    // Compute zi
    std::vector<double> z;
    double mean_z=0.0;
    for(int i=0; i< n; i++){
        double calc = (cluster.at(i).x*cluster.at(i).x)+(cluster.at(i).y*cluster.at(i).y);
        mean_z += calc;
        z.push_back(calc);
    }
    // Compute the mean of z
    mean_z = mean_z/n;
    //Form the data matrix from the n data points
    arma::mat Z=arma::mat(n, 4, arma::fill::zeros);
    for(int i=0; i< n; i++){
        Z(i,0)=z.at(i);
        Z(i,1)=cluster.at(i).x;
        Z(i,2)=cluster.at(i).y;
        Z(i,3)=1.0;
    }
    //Form the constraint matrix for the "Hyperaccurate algebraic fit" 
    arma::mat H=arma::mat(4, 4, arma::fill::eye);
    H(0,0)=8.0*mean_z;
    H(3,0)=2.0;
    H(3,3)=0.0;
    H(0,3)=2.0;
    //Compute H−1
    arma::mat H_inv=arma::mat(4, 4, arma::fill::zeros);
    H_inv(0,0)=0.0;
    H_inv(1, 1)=1.0;
    H_inv(2,2)=1.0;
    H_inv(3,0)=0.5;
    H_inv(3,3)=-2.0*mean_z;
    H_inv(0,3)=0.5;
    //compute the Singular Value Decomposition of Z
    arma::mat V=arma::mat(4, 4, arma::fill::zeros);
    arma::mat U=arma::mat(n, 4, arma::fill::zeros);
    arma::mat SIG=arma::mat(4, 4, arma::fill::zeros);
    arma::vec s=arma::vec(4, arma::fill::zeros);
    arma::svd(U, s, V, Z);
    double sigma_4 = s.min();
    arma::vec A;
    if(sigma_4<1e-12){
    //If the smallest singular value σ4 is less than 10−12, 
    // then Let A be the the 4th column of the V matrix
        A= {V(0,3), V(1,3), V(2,3), V(3,3)};
    }
    else{
        for(int i=0; i<4; i++){
            SIG(i,i) = s.at(i);
        }
        arma::mat Y=V*SIG*trans(V);
        arma::mat Q=Y*H_inv*Y;
        arma::cx_mat eigvec;
        arma::cx_vec eigval;
        arma::eig_gen(eigval, eigvec, Q);
        double min = 0.0;
        for(int i=0; i< 3; i++){
            if (eigval.at(i).real() > 0.0 && eigval.at(i+1).real() > 0.0){
                if(eigval.at(i).real() <= eigval.at(i+1).real()){
                    min = i;
                }
                else{
                    min = i+1;
                }
            }
            else if(eigval.at(i).real() > 0.0){
                min = i;
            }
            else if(eigval.at(i+1).real() > 0.0){
                min = i+1;
            }
        }
        arma::vec Astar = {eigvec(0,min).real(), eigvec(1,min).real(), eigvec(2,min).real(), eigvec(3,min).real()};
        A = inv(Y)*Astar;
    }
    Coord c;
    c.x = -A(1)/(2.0*A(0));
    c.y = -A(2)/(2.0*A(0));
    double r= std::sqrt(((A(1)*A(1)) + (A(2)*A(2))-4.0*(A(0)*A(3)))/(4.0*A(0)*A(0)));
    return std::vector<double>{c.x+center.x, c.y+center.y, r};
}


