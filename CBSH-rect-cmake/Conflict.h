#pragma once


#include <memory>
#include "common.h"
#include <string>
#include <sstream>
#include "MDDNode.h"
using namespace std;




// <loc, -1, t, VERTEX>
// <from, to, t, EDGE> 
// <B1, B2, t, RECTANGLE>
// <loc, t1, t2, CORRIDOR> 
// <loc, agent_id, t, TARGET>: path of agent_id should be of length at most t, and any other agent cannot be at loc at or after timestep t
// <-1, agent_id, t>: path of agent_id should be of length at least t + 1 


std::ostream& operator<<(std::ostream& os, const Constraint& constraint);



bool addGeneralKVerticalBarrierConstraint( int y, int Ri_x, int Rg_x, int Rg_t, int num_col, int St,
                                          std::list<Constraint>& constraints, int k,int extended, const MDDLevels* kMDD);

bool addGeneralKHorizontalBarrierConstraint(int x, int Ri_y, int Rg_y, int Rg_t, int num_col, int St,
                                            std::list<Constraint>& constraints, int k,int extended, const MDDLevels* kMDD);




class Conflict
{
public:

    // row conflict contains following information
	int a1;
	int a2;
	int t;
	int delta=0;
	int v1=0;
	int v2=-1;
	bool train_conflict = false;


	//Additional information for advanced conflict
	int k=0;
	int rs=0;
	int rg=0;
	int t_sg;

	bool isChasing = false;
	std::list<Constraint> constraint1;
	std::vector<std::list<Constraint>> multiConstraint1;
	std::list<Constraint> constraint2;
	std::vector<std::list<Constraint>> multiConstraint2;

	conflict_type type;
	conflict_priority p = conflict_priority::UNKNOWN;

	//other infor for repeatance check
	int info1 = 0;
	int info2 = 0;
	int info3 = 0;
	int info4 = 0;

	Conflict() {};
	Conflict(int v,int k,int t) {
		this->v1 = v;
		this->v2 = -1;
		this->k = k;
		this->t = t;
	};

	Conflict(int a1, int a2, int v1, int v2, int t, int delta, bool train_conflict ){
	    this->a1 = a1;
	    this->a2 = a2;
	    this->v1 = v1;
	    this->v2 = v2;
	    this->t = t;
	    this->delta = delta;
	    this->train_conflict = train_conflict;
	}


	void clear(){
        constraint1.clear();
        constraint2.clear();
        multiConstraint1.clear();
        multiConstraint2.clear();
	}


	void vertexTrainConflict(int a1, int a2, int v, int t,bool k2,bool lltp_only, bool target = false)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->v1 = v;
		this->v2 = -1;
		this->train_conflict = true;

		if (target){
		    this->constraint1.emplace_back(v, a1, t, constraint_type::NOPARKING); //place a parking constraint on it.
		    this->constraint1.emplace_back(v, -1, t, constraint_type::TRAIN_VERTEX);
		}
		if (lltp_only){
		    this->constraint1.emplace_back(v, -1, t, constraint_type::TRAIN_VERTEX);
		}
		else{
		    this->constraint1.emplace_back(v, t-delta, t-delta+k2, constraint_type::RANGE);
		}
		this->constraint2.emplace_back(v, -1, t, constraint_type::TRAIN_VERTEX);
		type = conflict_type::STANDARD;
	}

	void parkingConflict(int a1, int a2, int v, int t)
	{
	    this->a1 = a1;
	    this->a2 = a2;
	    this->t = t;
	    this->k = 0;
	    this->v1 = v;
	    this->v2 = -1;
	    this->train_conflict = true;



	    this->constraint1.emplace_back(v, a1, t, constraint_type::NOPARKING); // a1 must not part at v from 0, t
	    this->constraint2.emplace_back(v, a1, t, constraint_type::PARKING); // a1 must park at v from [0,t], all others not use v after t
	    type = conflict_type::TARGET;
	}

//    void vertexTrainConflictRange(int a1, int a2, int v, int t,int k=0,int range =0)
//    {
//        this->a1 = a1;
//        this->a2 = a2;
//        this->t = t;
//        this->k = k;
//        this->v1 = v;
//        this->v2 = -1;
//        this->train_conflict = true;
//        for (int i = 0; i <= range; i++) {
//            this->constraint1.emplace_back(v, -1, t+i, constraint_type::TRAIN_VERTEX);
//            this->constraint2.emplace_back(v, -1, t+i, constraint_type::TRAIN_VERTEX);
//        }
//        type = conflict_type::STANDARD;
//    }

    void selfConflict(int a)
    {
        this->a1 = a;
        this->a2 = a;
        this->constraint1.emplace_back(-1, -1, -1, constraint_type::TRAIN_VERTEX);
        this->train_conflict = true;
        type = conflict_type::SELF_CONFLICT;
    }

    void vertexConflict(int a1, int a2, int v, int t,int delta=0, int k1 = 0,int k2 =0)
    {
        this->a1 = a1;
        this->a2 = a2;
        this->t = t;
        this->k = delta;
        this->v1 = v;
        this->v2 = -1;
        this->constraint1.emplace_back(v, t, t + k2, constraint_type::RANGE);
        this->constraint2.emplace_back(v, t, t + k1, constraint_type::RANGE);
        type = conflict_type::STANDARD;
    }

    void vertexConflictM2(int a1, int a2, int v, int t1, int t2,int delta=0, int k1 = 0,int k2 =0)
    {
	    //a1 is the agent arrive v earlier
	    this->a1 = a1;
	    this->a2 = a2;
	    this->t = t;
	    this->k = delta;
	    this->v1 = v;
	    this->v2 = -1;
        this->constraint1.emplace_back(v, t1, t2 + k2, constraint_type::RANGE);
        this->constraint2.emplace_back(v, t2, t1 + k1, constraint_type::RANGE);
	    type = conflict_type::STANDARD;
    }
		
	void edgeConflict(int a1, int a2, int v1, int v2, int t)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = 0;
		this->v1 = v1;
		this->v2 = v2;

		this->constraint1.emplace_back(v1, v2, t, constraint_type::EDGE);
		this->constraint2.emplace_back(v2, v1, t, constraint_type::EDGE);
		type = conflict_type::STANDARD;
	}



	// t3 
	void corridorConflict(int a1, int a2, int v1, int v2, int t3, int t4, int t3_, int t4_,int t1, int t2, int l,int k_1, int k_2)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->k = l;
		this->t = std::min(t3, t4);
		this->v1 = v1;
		this->v2 = v2;
		this->info1 = t3;
		this->info2 = t4;
		this->info3 = std::min( std::max(t2 + k_2, t3_ - 1) , t4 + l + k_2);
		this->info4 = std::min( std::max(t1 + k_1, t4_ - 1) , t3 + l + k_1);
		//k is corridor length
		this->constraint1.emplace_back(v1, t3, std::min( std::max(t2 + k_2, t3_ - 1) , t4 + l + k_2), constraint_type::RANGE);
		this->constraint2.emplace_back(v2, t4, std::min( std::max(t1 + k_1, t4_ - 1) , t3 + l + k_1), constraint_type::RANGE);
		type = conflict_type::CORRIDOR2;
	}

    int generalKRectangleConflict(int a1, int a2, const std::pair<int, int>& Rs, const std::pair<int, int>& Rg,
                            const std::pair<int, int>& s1, const std::pair<int, int>& s2, int rt1,int rt2,
                            int S1_t, int S2_t, const std::pair<int, int>& G1, const std::pair<int, int>& G2,
                            int num_col, int a1k, int a2k,int a1_extended,int a2_extended, const MDDLevels* a1kMDD, const MDDLevels* a2kMDD) // For K-RM
    {
	    //s and g are B and A in paper
	    //Rs is D and Rg is E
        this->a1 = a1;
        this->a2 = a2;
        this->t_sg = rt1;
        this->rs = Rs.first*num_col + Rs.second;
        this->rg = Rg.first*num_col + Rg.second;

        int s1_x = s1.first;
        int s1_y = s1.second;
        int s2_x = s2.first;
        int s2_y = s2.second;
        int Rg_x = Rg.first;
        int Rg_y = Rg.second;
        int g1_x = G1.first;
        int g1_y = G1.second;
        int g2_x = G2.first;
        int g2_y = G2.second;



        int a1Rg = S1_t + getMahattanDistance(s1_x, s1_y, Rg_x, Rg_y);
        int a1RgBypass = a1Rg + 2 * (getMahattanDistance(s2_x, s2_y, Rs.first, Rs.second) + 1);

        int a2Rg = S2_t + getMahattanDistance(s2_x, s2_y, Rg_x, Rg_y);
        int a2RgBypass = a2Rg + 2 * (getMahattanDistance(s1_x, s1_y, Rs.first, Rs.second) + 1);


        int R1_x, R1_y, R2_x, R2_y, G1_x, G1_y, G2_x, G2_y, G1_t, G2_t,E1_t,E2_t;

        //if condition for classify different s g (B E) layout to handle non-cardinal and semi-cardinal cases.

        if ((
                (((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) && ((s1_x - g1_x) * (g1_x - g2_x) > 0 || (s1_y - g1_y) * (g1_y - g2_y) < 0))
                ||
                (((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) && ((s2_x - g2_x) * (g2_x - g1_x) < 0 || (s2_y - g2_y) * (g2_y - g1_y) > 0))
        )) { // s1 always in the middle,s2 always between s1 and g1 && g1 lies right side of s1 s2 g2 line
            // or s2 always in the middle, s1 always between s2 and g2 && g2 lies left side of s2 s1 g1 line
            int sign_a1 = g1_y - s1_y >= 0 ? 1 : -1;
            int sign_a2 = g2_x - s2_x >= 0 ? 1 : -1;

            int a1_exit = Rg_y + sign_a1 * a1_extended;
			int a1_entrance = Rs.second - sign_a1 * a1_extended;


            int a2_exit = Rg_x + sign_a2 * a2_extended;
			int a2_entrance = Rs.first - sign_a2 * a2_extended;

            if (sign_a1 * a1_entrance < sign_a1 * s1_y || sign_a1 * a1_exit > sign_a1 * g1_y){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_x || sign_a2 * a2_exit > sign_a2 * g2_x){
                return 1;
            }


            R1_x = Rs.first;
            G1_x = Rg_x;

            R2_y = Rs.second;
            G2_y = Rg_y;

            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, G1_x, a1_exit);
            E1_t = rt1 + getMahattanDistance(Rs.first, a1_entrance, G1_x, a1_entrance);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, a2_exit,G2_y);
            E2_t = rt2 + getMahattanDistance(a2_entrance, Rs.second, a2_entrance, G2_y);


            std::list<Constraint> constraint11;
            addGeneralKVerticalBarrierConstraint( a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint11);


            //chasing case always 4 way split
            std::list<Constraint> constraint12;
            addGeneralKVerticalBarrierConstraint( a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint12);


            std::list<Constraint> constraint21;
            addGeneralKHorizontalBarrierConstraint( a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint21);

            //chasing case always 4 way split
            std::list<Constraint> constraint22;
            addGeneralKHorizontalBarrierConstraint( a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint22);
            this->isChasing = true;


        }
        else if ((
                (((s1_x - s2_x) * (s2_x - g2_x) < 0 && (s1_y - s2_y) * (s2_y - g2_y) < 0) && ((s2_x - g2_x) * (g2_x - g1_x) > 0 || (s2_y - g2_y) * (g2_y - g1_y) < 0))
                ||
                (((s2_x - s1_x) * (s1_x - g1_x) < 0 && (s2_y - s1_y) * (s1_y - g1_y) < 0) && ((s1_x - g1_x) * (g1_x - g2_x) < 0 || (s1_y - g1_y) * (g1_y - g2_y) > 0))
        ))
        {
            // s2 always in the middle, s1 always between s2 and g2 && g2 lies right side of s2 s1 g1 line
            //or s1 always in the middle,s2 always between s1 and g1 && g1 lies left side of s1 s2 g2 line

            int sign_a1 = g1_x - s1_x >= 0 ? 1 : -1;
            int sign_a2 = g2_y - s2_y >= 0 ? 1 : -1;

            int a1_exit = Rg_x + sign_a1 * a1_extended;
			int a1_entrance = Rs.first - sign_a1 * a1_extended;


            int a2_exit = Rg_y + sign_a2 * a2_extended;
			int a2_entrance = Rs.second - sign_a2 * a2_extended;

            if (sign_a1 * a1_entrance < sign_a1 * s1_x || sign_a1 * a1_exit > sign_a1 * g1_x){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_y || sign_a2 * a2_exit > sign_a2 * g2_y){
                return 1;
            }

            R2_x = Rs.first;
            G2_x = Rg_x;

            R1_y = Rs.second;
            G1_y = Rg.second;


            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, a1_exit, G1_y);
            E1_t = rt1 + getMahattanDistance(a1_entrance, Rs.second, a1_entrance, G1_y);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, G2_x, a2_exit);
            E2_t = rt2 + getMahattanDistance(Rs.first, a2_entrance, G2_x, a2_entrance);


            std::list<Constraint> constraint11;

            addGeneralKHorizontalBarrierConstraint( a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint11);

            //chasing case always 4 way split
            std::list<Constraint> constraint12;
            addGeneralKHorizontalBarrierConstraint( a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint12);




            std::list<Constraint> constraint21;
            addGeneralKVerticalBarrierConstraint( a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint21);

            //chasing case always 4 way split
            std::list<Constraint> constraint22;
            addGeneralKVerticalBarrierConstraint( a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint22);

            this->isChasing = true;


        }
        else if ((s1_x == s2_x && (s1_y - s2_y) * (s2_y - Rg_y) < 0) ||
                 (s1_x != s2_x && (s1_x - s2_x)*(s2_x - Rg_x) >= 0))
        {


            int sign_a1 = g1_x - s1_x >= 0 ? 1 : -1;
            int sign_a2 = g2_y - s2_y >= 0 ? 1 : -1;

            int a1_exit = Rg_x + sign_a1 * a1_extended;
			int a1_entrance = Rs.first - sign_a1 * a1_extended;


            int a2_exit = Rg_y + sign_a2 * a2_extended;
			int a2_entrance = Rs.second - sign_a2 * a2_extended;

            if (sign_a1 * a1_entrance < sign_a1 * s1_x || sign_a1 * a1_exit > sign_a1 * g1_x){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_y || sign_a2 * a2_exit > sign_a2 * g2_y){
                return 1;
            }


            R2_x = Rs.first;
            G2_x = Rg_x;

            R1_y = Rs.second;
            G1_y = Rg.second;


            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, a1_exit, G1_y);
            E1_t = rt1 + getMahattanDistance(a1_entrance, Rs.second, a1_entrance, G1_y);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, G2_x, a2_exit);
            E2_t = rt2 + getMahattanDistance(Rs.first, a2_entrance, G2_x, a2_entrance);


            std::list<Constraint> constraint11;

            addGeneralKHorizontalBarrierConstraint(a1_exit, R1_y, G1_y, G1_t, num_col, S1_t, constraint11, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint11);

            std::list<Constraint> constraint12;
            addGeneralKHorizontalBarrierConstraint( a1_entrance, R1_y, G1_y, E1_t, num_col, S1_t, constraint12, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint12);




            std::list<Constraint> constraint21;
            addGeneralKVerticalBarrierConstraint( a2_exit, R2_x, G2_x, G2_t, num_col, S2_t, constraint21, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint21);

            std::list<Constraint> constraint22;
            addGeneralKVerticalBarrierConstraint(a2_entrance, R2_x, G2_x, E2_t, num_col, S2_t, constraint22, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint22);



        }
        else
        {

            int sign_a1 = g1_y - s1_y >= 0 ? 1 : -1;
            int sign_a2 = g2_x - s2_x >= 0 ? 1 : -1;

            int a1_exit = Rg_y + sign_a1 * a1_extended;
			int a1_entrance = Rs.second - sign_a1 * a1_extended;


            int a2_exit = Rg_x + sign_a2 * a2_extended;
			int a2_entrance = Rs.first - sign_a2 * a2_extended;

            if (sign_a1 * a1_entrance < sign_a1 * s1_y || sign_a1 * a1_exit > sign_a1 * g1_y){
                return 2;
            }
            if (sign_a2 * a2_entrance < sign_a2 * s2_x || sign_a2 * a2_exit > sign_a2 * g2_x){
                return 1;
            }

            R1_x = Rs.first;
            G1_x = Rg_x;

            R2_y = Rs.second;
            G2_y = Rg_y;

            G1_t = rt1 + getMahattanDistance(Rs.first, Rs.second, G1_x, a1_exit);
            E1_t = rt1 + getMahattanDistance(Rs.first, a1_entrance, G1_x, a1_entrance);
            G2_t = rt2 + getMahattanDistance(Rs.first, Rs.second, a2_exit, G2_y);
            E2_t = rt2 + getMahattanDistance(a2_entrance, Rs.second, a2_entrance, G2_y);


            std::list<Constraint> constraint11;
            addGeneralKVerticalBarrierConstraint( a1_exit, R1_x, G1_x, G1_t, num_col, S1_t, constraint11, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint11);


            std::list<Constraint> constraint12;
            addGeneralKVerticalBarrierConstraint( a1_entrance, R1_x, G1_x, E1_t, num_col, S1_t, constraint12, a1k,a2_extended, a1kMDD);
            multiConstraint1.push_back(constraint12);



            std::list<Constraint> constraint21;
            addGeneralKHorizontalBarrierConstraint( a2_exit, R2_y, G2_y, G2_t, num_col, S2_t, constraint21, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint21);

            std::list<Constraint> constraint22;
            addGeneralKHorizontalBarrierConstraint( a2_entrance, R2_y, G2_y, E2_t, num_col, S2_t, constraint22, a2k,a1_extended, a2kMDD);
            multiConstraint2.push_back(constraint22);


        }
        type = conflict_type::RECTANGLE4;
        return 0;
    }


	void targetConflict(int a1, int a2, int v, int t,int k_2)
	{
		this->a1 = a1;
		this->a2 = a2;
		this->t = t;
		this->k = 0;
		this->v1 = v;
		this->v2 = -1;


		this->constraint1.emplace_back(-1, a1, t + k_2 /* kDelay>0? t + kDelay+1:t*/, constraint_type::LENGTH); // length of a1 should be larger than t
		this->constraint2.emplace_back(v, a1, t + k_2, constraint_type::LENGTH); // length of a1 should be no larger than t, and other agents can not use v at and after timestep t
		type = conflict_type::TARGET;
	}

	//	void corridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int k, int h, int kRobust)
	//	{
	//		this->a1 = a1;
	//		this->a2 = a2;
	//		this->t = std::min(t1, t2);
	//		this->v1 = v1;
	//		this->v2 = v2;
	//		this->k = k;
	//		this->constraint1.emplace_back(v1, t1, t1 + 2 * k - 1 + kRobust, constraint_type::RANGE);
	//		this->constraint1.emplace_back(v2, t1 + k, std::min(t2 + 2 * k, t1 + h - 1) + kRobust, constraint_type::RANGE);
	//		this->constraint2.emplace_back(v2, t2, t2 + 2 * k - 1 + kRobust, constraint_type::RANGE);
	//		this->constraint2.emplace_back(v1, t2 + k, std::min(t1 + 2 * k, t2 + h - 1) + kRobust, constraint_type::RANGE);
	//		type = conflict_type::CORRIDOR4;
	//	}

//	void trainCorridorConflict(int a1, int a2, int v1, int v2, int t1, int t2, int e1, int e2, int k, int kRobust)
//	{
//	    this->a1 = a1;
//	    this->a2 = a2;
//	    this->k = k;
//	    this->t = std::min(e1, e2);
//	    this->v1 = v1;
//	    this->v2 = v2;
//	    this->constraint1.emplace_back(v1, t2, e2-1 + kRobust, constraint_type::RANGE);
//	    this->constraint2.emplace_back(v2, t1, e1-1 + kRobust, constraint_type::RANGE);
//	    type = conflict_type::CORRIDOR2;
//	}


};

std::ostream& operator<<(std::ostream& os, const Conflict& conflict);

bool operator < (const Conflict& conflict1, const Conflict& conflict2);





