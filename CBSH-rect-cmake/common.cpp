#include "common.h"
#include <iostream>


//std::ostream& operator<<(std::ostream& os, const RConflict& conflict)
//{
//	os << "<" << std::get<0>(conflict) << "," << std::get<1>(conflict) << "," <<
//		 - std::get<2>(conflict) - 1 << "," <<  - std::get<3>(conflict) - 1<< "," <<
//		- std::get<4>(conflict) << ">";
//	return os;
//}

//std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
//{
//	if (std::get<4>(conflict) < 0)
//		os << "<" << std::get<0>(conflict) << "," << std::get<1>(conflict) << "," <<
//			-std::get<2>(conflict) - 1 << "," << -std::get<3>(conflict) - 1 << "," <<
//			-std::get<4>(conflict) << ">";
//	else
//		os << "<" << std::get<0>(conflict) << "," << std::get<1>(conflict) << "," <<
//		std::get<2>(conflict) << "," << std::get<3>(conflict) << "," <<
//		std::get<4>(conflict) << ">";
//	return os;
//}


int getLocation(const std::vector<PathEntry>& path, int timestep)
{
	if (timestep >= path.size())
		return path.back().location;
	else
		return path[timestep].location;
}