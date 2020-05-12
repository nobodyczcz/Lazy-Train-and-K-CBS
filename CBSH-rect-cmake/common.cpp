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

int getMahattanDistance(int loc1_x, int loc1_y, int loc2_x, int loc2_y)
{
	return std::abs(loc1_x - loc2_x) + std::abs(loc1_y - loc2_y);
}

int getArea(int loc1_x, int loc1_y, int loc2_x, int loc2_y){
    return std::abs(loc1_x - loc2_x) * std::abs(loc1_y - loc2_y);
}

int getAction(int loc, int pre_loc,int num_col) {
	int difference = loc - pre_loc;
	if (difference == 0) {
		return action::WAIT;
	}
	else if (difference == 1) {
		return action::RIGHT;
	}
	else if (difference == -1) {
		return action::LEFT;
	}
	else if (difference == num_col) {
		return action::DOWN;
	}
	else if (difference == - num_col) {
		return action::UP;
	}
	else {
		return -1;
	}
}