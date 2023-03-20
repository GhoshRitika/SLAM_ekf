#ifndef CIRCLEFITTING_INCLUDE_GUARD_HPP
#define CIRCLEFITTING_INCLUDE_GUARD_HPP
/// \file
/// \brief

#include <vector>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
    ///\brief saves the x and y positions of a coordinate
     struct Coord
     {
          /// \brief the x coord
          double x=0.0;

          /// \brief the y coord
          double y=0.0;
     };

   /// \brief fit a cluster of pointers into a circle
   /// \param It is avector of coordinates of all points in the cluster
   /// \return the radii and center of the circle
   std::vector<double> fit_circle(std::vector<Coord> &cluster_real);
}

#endif