#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief models the kinematics of a differential drive robot with a given wheel dimensions.

#include "turtlelib/rigid2d.hpp"


namespace turtlelib

{

     ///\brief Stores the angles of 2 wheels in differential drive robot
     struct Wheel_ang
     {
          /// \brief the angle of the right wheel
          double right=0.0;

          /// \brief the angle of the left wheel
          double left=0.0;
     };

     ///\brief Stores the configuration of the robot
     struct Config
     {
          /// \brief the angle of the robot body
          double theta;

          /// \brief the x position of the robot
          double x;

          /// \brief the y position of the robot
          double y;
     };


     /// \brief track the robots wheel positions and configuration
     class DiffDrive
     {
     private:

          Config q;
          Wheel_ang phi;
          double r, D;

     public:

          /// \brief Default constructor
          DiffDrive();


          /// \brief define radius and track of the robot wheel
          /// \param rad - the radius of robot wheel
          /// \param track - the wheel track of robot
          explicit DiffDrive(double rad, double track);


          /// \brief define all the parameters of the robot
          /// \param c - the initial configuration of the robot
          /// \param w - The wheel positions, i.e angles 
          /// \param rad - the radius of robot wheel
          /// \param track - the wheel track of robot 
          explicit DiffDrive(Config c, Wheel_ang w, double rad, double track);


          /// \brief gets the current configuration of the robot
          /// \return the current configuration of the robot
          Config get_config() const;

          /// \brief gets the current wheel angles of the robot
          /// \return the current wheel angles of the robot
          Wheel_ang get_phi() const;

          /// \brief sets the current wheel angles of the robot
          void set_phi(Wheel_ang phi_new);

          /// \brief sets the current configuration of the robot
          void set_config(Config config);


          /// \brief update the robot configuration
          /// \param phi_new The given new angles of the wheel
          Twist2D FwdK(Wheel_ang phi_new);


          /// \brief Calculates the wheel velocities corresponding to the twist
          /// \param Vb the twist of the robot
          /// \return the calculated wheel velocities
          Wheel_ang InvK (Twist2D Vb);
     };
}

#endif