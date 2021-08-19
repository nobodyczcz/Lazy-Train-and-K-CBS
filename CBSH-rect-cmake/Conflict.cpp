#include "Conflict.h"

std::ostream& operator<<(std::ostream& os, const Constraint& constraint)
{
	os << "<" << std::get<0>(constraint) << "," << std::get<1>(constraint) << "," <<
		std::get<2>(constraint) << "," << std::get<3>(constraint) << ">";
	return os;
}


std::ostream& operator<<(std::ostream& os, const Conflict& conflict)
{
    switch (conflict.train_conflict)
    {
        case true:
            os <<"Train ";
            break;
        case false:
            os <<"K ";
            break;
    }
	switch (conflict.p)
	{
		case conflict_priority::CARDINAL:
			os << "cardinal ";
			break;
		case conflict_priority::SEMI:
			os << "semi-cardinal ";
			break;
		case conflict_priority::NON:
			os << "non-cardinal ";
			break;
	}

	switch (conflict.type)
	{
		case conflict_type::STANDARD:
            os << "standard";
            break;
        case conflict_type::SELF_CONFLICT :
            os << "self";
            break;
		case conflict_type::CORRIDOR2:
			os << "corrdior2";
			break;
		case conflict_type::CORRIDOR4:
			os << "corrdior4";
			break;
		case conflict_type::TARGET:
			os << "target";
	}

	os << " conflict:  " << conflict.a1 << " with ";
	for (auto con : conflict.constraint1)
		os << con << ",";	
	for (auto cons : conflict.multiConstraint1) {
		os << " node: ";
		for (auto con1 : cons)
			os << con1 << ",";
	}


	os << " and " << conflict.a2 << " with ";
	for (auto con : conflict.constraint2)
		os << con << ",";	

	for (auto cons : conflict.multiConstraint2) {
		os << " node: ";
		for (auto con1 : cons)
			os << con1 << ",";
	}
	os << std::endl;
	return os;
}

bool operator < (const Conflict& conflict1, const Conflict& conflict2) // return true if conflict2 has higher priority
{

    if (conflict1.train_conflict && !conflict2.train_conflict )
        return true;
    else if (!conflict1.train_conflict && conflict2.train_conflict )
        return false;



    if (conflict1.p < conflict2.p)
		return false;
	else if (conflict1.p > conflict2.p)
		return true;
	else if (conflict1.p == conflict_priority::CARDINAL) // both are cardinal
	{
        if (conflict1.type == conflict_type::RECTANGLE4 || conflict1.type == conflict_type::RECTANGLE){
            if (conflict2.type != conflict_type::RECTANGLE4 || conflict2.type != conflict_type::RECTANGLE)
                return false;
	    }
	    else if (conflict2.type == conflict_type::RECTANGLE4 || conflict2.type == conflict_type::RECTANGLE){
	        return true;
	    }
        else if (conflict1.type == conflict_type::CORRIDOR2)
		{
			if (conflict2.type != conflict_type::CORRIDOR2)
				return false;
		}
        else if (conflict2.type == conflict_type::CORRIDOR2 ) {
            return true;
        }
        else if (conflict1.type == conflict_type::TARGET ){
            if (conflict2.type != conflict_type::TARGET )
                return false;
        }
        else if (conflict2.type == conflict_type::TARGET ) {
            return true;
        }


	}
	else // both are semi or both are non 
	{
        if (conflict1.type == conflict_type::RECTANGLE4 || conflict1.type == conflict_type::RECTANGLE){
            if (conflict2.type != conflict_type::RECTANGLE4 || conflict2.type != conflict_type::RECTANGLE)
                return false;
        }
        else if (conflict2.type == conflict_type::RECTANGLE4 || conflict2.type == conflict_type::RECTANGLE){
            return true;
        }
        if (conflict2.type == conflict_type::CORRIDOR2 &&  conflict1.type != conflict_type::CORRIDOR2)
		{
			return true;
		}
		else if (conflict2.type != conflict_type::CORRIDOR2 &&  conflict1.type == conflict_type::CORRIDOR2)
		{
			return false;
		}
        else if (conflict2.type == conflict_type::CORRIDOR2 ) {
            return true;
        }
        else if (conflict1.type == conflict_type::TARGET ){
            if (conflict2.type != conflict_type::TARGET )
                return false;
        }
        else if (conflict2.type == conflict_type::TARGET ) {
            return true;
        }


	}



	if (conflict2.t < conflict1.t)
	{
		return true;
	}
	else
		return false;
}


// add a vertival modified barrier constraint
bool addGeneralKVerticalBarrierConstraint( int y,
                                              int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
                                              std::list<Constraint>& constraints, int k,int extended, const MDDLevels* kMDD)
{


    int sign = Ri_x < Rg_x ? 1 : -1;
    int Ri_t = Rg_t - abs(Ri_x - Rg_x);

    for (int t2 = Ri_t; t2 <= Rg_t; t2++)
    {
        int loc = (Ri_x + (t2 - Ri_t) * sign) * num_col + y;
//        std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
        for (int i = 0; i <= k; i++) {
            if (t2 + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t2 + i)){
                    if (node->locs.front() == loc){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
//                    std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i << "|";
                }
            }
        }
//        std::cout <<endl;

    }

    //add constraint for extended area

    for (int extend = 1;  extend <= extended; extend++){
        int t_left = Ri_t + extend;
        int t_right = Rg_t + extend;
        int range = k - extend * 2;
        int loc_left = (Ri_x - extend * sign) * num_col + y;
        int loc_right = (Rg_x + extend * sign) * num_col + y;
        if (range < 0){
            break;
        }

        for (int i = 0; i <= range; i++) {

            if (t_left + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_left + i)){
                    if (node->locs.front() == loc_left){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_left, -1, t_left + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }

            if (t_right + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_right + i)){
                    if (node->locs.front() == loc_right){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_right, -1, t_right + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }
        }

    }

    if (constraints.empty())
    {
        // std::cout << "Fail to add modified barrier constraints!" << std::endl;
        return false;
    }
    else
        return true;
}


// add a horizontal modified barrier constraint
bool addGeneralKHorizontalBarrierConstraint( int x,
                                                int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
                                                std::list<Constraint>& constraints, int k,int extended, const MDDLevels* kMDD)
{

    //add constraint for main area
    int sign = Ri_y < Rg_y ? 1 : -1;
    int Ri_t = Rg_t - abs(Ri_y - Rg_y);
    for (int t2 = Ri_t; t2 <= Rg_t; t2++)
    {
        int loc = (Ri_y + (t2 - Ri_t) * sign) + x * num_col;
//        std::cout << "target loc: " << loc / num_col << "," << loc % num_col << std::endl;
        for (int i = 0; i <= k; i++) {

            if (t2 + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t2 + i)){
                    if (node->locs.front() == loc){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc, -1, t2 + i , constraint_type::VERTEX); // add constraints [t1, t2]
//                        std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i << "|";
                }
            }

        }
//        std::cout <<endl;

    }

    //add constraint for extended area
    for (int extend = 1;  extend <= extended; extend++){
        int t_left = Ri_t + extend;
        int t_right = Rg_t + extend;
        int range = k - extend * 2;
        int loc_left = (Ri_y - extend * sign) + x * num_col ;
        int loc_right = (Rg_y + extend * sign) + x * num_col ;
        if (range < 0){
            break;
        }

        for (int i = 0; i <= range; i++) {

            if (t_left + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_left + i)){
                    if (node->locs.front() == loc_left){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_left, -1, t_left + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }

            if (t_right + i < kMDD->size()){
                bool find = false;
                for (auto node : kMDD->at(t_right + i)){
                    if (node->locs.front() == loc_right){
                        find = true;
                        break;
                    }
                }
                if (find) {
                    constraints.emplace_back(loc_right, -1, t_right + i , constraint_type::VERTEX); // add constraints [t1, t2]
                    //std::cout << "kmdd loc: " << loc / num_col << "," << loc % num_col << " t: " << t2 + i + consk << "|";
                }
            }
        }

    }


    if (constraints.empty())
    {
        // std::cout << "Fail to add modified barrier constraints!" << std::endl;
        return false;
    }
    else
        return true;
}





