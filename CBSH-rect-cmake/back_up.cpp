//
// Created by Zhe Chen on 22/8/20.
//


//Rectangle reasoning for semi and non cardinal vertex conflicts
if (cons_strategy == constraint_strategy::CBSH_CR)//Identify cardinal rectangle by start and goals
{
if (isRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2],
paths[a1]->size() - 1, paths[a2]->size() - 1) &&
classifyRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2]) == 2)
{
std::pair<int, int> Rg = getRg(al.initial_locations[a1], al.goal_locations[a1], al.goal_locations[a2]);
std::pair<int, int> Rs = getRs(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1]);
int Rg_t = con->t + abs(Rg.first - loc1 / num_col) + abs(Rg.second - loc1 % num_col);

auto new_rectangle = std::shared_ptr<Conflict>(new Conflict(loc1,con->k,timestep));

int rt1, rt2;

if(option.RM4way>=2 && option.RM4way<=5){
if (con->k == 0) {
rt1 = timestep - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col);
rt2 = rt1;
}
else {
rt1 = timestep - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col);
rt2 = timestep2 - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col) - 1;
}
}
else{
rt1 = timestep - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col);
rt2 = rt1;
}

bool success;
int way = option.RM4way==1?6:4;
if(option.RM4way>=6){
way = option.RM4way;
}

if (screen >= 5) {
cout << "s1: " << al.initial_locations[a1].first << " " << al.initial_locations[a1].second << endl;
cout << "g1: " << al.goal_locations[a1].first << " " << al.goal_locations[a1].second << endl;
cout << "s1_t: " << 0 << endl;
cout << "rt1: " << rt1 << endl;


cout << "s2: " << al.initial_locations[a2].first << " " << al.initial_locations[a2].second << endl;
cout << "g2: " << al.goal_locations[a2].first << " " << al.goal_locations[a2].second << endl;
cout << "s2_t: " << 0 << endl;
cout << "rt2: " << rt2 << endl;

cout << "Rs: " << Rs.first << " " << Rs.second << endl;
cout << "Rg: " << Rg.first << " " << Rg.second << endl;
cout << "split setting: "<<way<<endl;
}

success = new_rectangle->kRectangleConflict(a1, a2, Rs, Rg,
                                            al.initial_locations[a1],
                                            al.initial_locations[a2],
                                            rt1, rt2, paths, 0, 0,
                                            al.goal_locations[a1],
                                            al.goal_locations[a2],
                                            num_col, kDelay, way, false);

bool isBlocked = true;

for (auto constraint : new_rectangle->multiConstraint1) {
isBlocked = isBlocked && blocked(*paths[new_rectangle->a1], constraint);
}
for (auto constraint : new_rectangle->multiConstraint2) {
isBlocked = isBlocked && blocked(*paths[new_rectangle->a2], constraint);
}

if (success && isBlocked) {
new_rectangle->p = conflict_priority::CARDINAL;
parent.conflicts.push_back(new_rectangle);
if(screen>=5){
cout<<"Add "<<*new_rectangle<<endl;
}
continue;
}
else if (screen >= 4) {
cout << "Not success or not blocked" << endl;
cout << "succcess " << success << endl;
cout << "isBlocked " << isBlocked << endl;
}
}
}
else if (cons_strategy == constraint_strategy::CBSH_R)//Identify rectangle by start and goals
{
//cout << "identify rectangle by start and goals" << endl;
//Identify rectangles by start and goals
if (isRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2],
paths[a1]->size() - 1, paths[a2]->size() - 1))
{
int type = classifyRectangleConflict(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1], al.goal_locations[a2]);
std::pair<int, int> Rg = getRg(al.initial_locations[a1], al.goal_locations[a1], al.goal_locations[a2]);
std::pair<int, int> Rs = getRs(al.initial_locations[a1], al.initial_locations[a2], al.goal_locations[a1]);
int Rg_t = con->t + abs(Rg.first - loc1 / num_col) + abs(Rg.second - loc1 % num_col);

auto new_rectangle = std::shared_ptr<Conflict>(new Conflict(loc1,con->k,timestep));
int rt1, rt2;
if (con->k == 0) {
rt1 = timestep - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col);
rt2 = rt1;
}
else {
rt1 = timestep - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col);
rt2 = timestep2 - getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1%num_col) - 1;
}

bool success;
success = new_rectangle->kRectangleConflict(a1, a2, Rs, Rg,
                                            al.initial_locations[a1],
                                            al.initial_locations[a2],
                                            rt1, rt2, paths, 0, 0,
                                            al.goal_locations[a1],
                                            al.goal_locations[a2],
                                            num_col, kDelay, 4, false);

bool isBlocked = true;

for (auto constraint : new_rectangle->multiConstraint1) {
isBlocked = isBlocked && blocked(*paths[new_rectangle->a1], constraint);
}
for (auto constraint : new_rectangle->multiConstraint2) {
isBlocked = isBlocked && blocked(*paths[new_rectangle->a2], constraint);
}

if (success && isBlocked)
{

if (type == 2)
new_rectangle->p = conflict_priority::CARDINAL;
else if (type == 1) // && !findRectangleConflict(parent.parent, *conflict))
new_rectangle->p = conflict_priority::SEMI;
else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
new_rectangle->p = conflict_priority::NON;
parent.conflicts.push_back(new_rectangle);
continue;
}
else if(screen>=4) {
cout << "Not success or not blocked" << endl;
cout << "succcess " << success << endl;
cout << "isBlocked " << isBlocked << endl;

}
}
}
else

else if (rectangleMDD )
{

//stringstream conString;
////con << *(con);
//if (con->k == 0) {
//	conString << min(con->a1, con->a2) << ",";
//	conString << max(con->a1, con->a2);
//}
//else {
//	conString << con->a1 << ",";
//	conString << con->a2;
//}
//conString << ",("
//	<< con->originalConf1 / num_col << ","
//	<< con->originalConf1 % num_col << ")" << ",("
//	<< con->originalConf2 / num_col << ","
//	<< con->originalConf2 % num_col << "),"
//	<< con->t << "," << con->k << "," << conflict_type::RECTANGLE4;
//ICBSNode* temp = &parent;

//bool repeat = false;
//while (temp != NULL) {
//	if (temp->resolvedConflicts.count(conString.str())) {
//		repeat = true;
//		break;
//	}
//	temp = temp->parent;
//}
//if (repeat) {
//	parent.conflicts.push_back(con);
//	continue;
//}



std::clock_t RM_Start = std::clock();

std::list<int>	s1s = getStartCandidates(*paths[a1], timestep, num_col);
std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep, num_col);
std::list<int>	s2s = getStartCandidates(*paths[a2], timestep2, num_col);
std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep2, num_col);


// Try all possible combinations
bool found = false;
std::shared_ptr<Conflict> rectangle; // = std::shared_ptr<Conflict>(new Conflict());
int type = -1;
int area = 0;
int distance = 0;
std::pair<int, int> Rgf;
std::pair<int, int> Rsf;
int t1_startf;
int t1_endf;
int t2_startf;
int t2_endf;
int Rg_tf;
int Rs_tf;
int rt1_f;
int rt2_f;
int s1f;
int s2f;
int g1f;
int g2f;
int flipTypef;
bool isChasing = false;
bool isChasingf = false;

int flipType=-1;
if (screen >= 4) {
cout << "try find rectangle" << endl;
cout << "Agents: " << *con << endl;
cout << "s1s length" << s1s.size() << endl;
cout << "s2s length" << s2s.size() << endl;

}

for (int t1_start : s1s)
{
for (int t1_end : g1s)
{

int s1 = paths[a1]->at(t1_start).location;
int g1 = paths[a1]->at(t1_end).location;
if (!isManhattanOptimal(s1, g1, t1_end - t1_start, num_col)) {
if (screen >= 5) {
cout << "s1: " << s1/num_col<<","<< s1%num_col << endl;
cout << "g1: " << g1 / num_col << "," << g1 % num_col << endl;
cout << "s1 g1 not optimal" << endl;
}
continue;
}

for (int t2_start : s2s)
{
for (int t2_end : g2s)
{
isChasing = 0;

int s2 = paths[a2]->at(t2_start).location;
int g2 = paths[a2]->at(t2_end).location;
if (!isManhattanOptimal(s2, g2, t2_end - t2_start, num_col)) {
if (screen >= 5) {
cout << "s2: " << s2 / num_col << "," << s2 % num_col << endl;
cout << "g2: " << g2 / num_col << "," << g2 % num_col << endl;
cout << "s2 g2 not optimal" << endl;
}

continue;
}

if (option.flippedRec) {
int flipped = isFlippedRectangleConflict(s1, s2, g1, g2, num_col);
if (flipped <= -1 || (flipped>=1 && kDelay==0)) //flipped <= -1 means not a rectangle.
//0 means rectangle with no flip. 1 is 1 flip. 2 is 2 flip.
{
if (screen >=5) {
cout << "s1: " << s1 / num_col << " " << s1 % num_col << endl;
cout << "g1: " << g1 / num_col << " " << g1 % num_col << endl;
cout << "s1_t: " << t1_start << endl;

cout << "s2: " << s2 / num_col << " " << s2 % num_col << endl;
cout << "g2: " << g2 / num_col << " " << g2 % num_col << endl;
cout << "s2_t: " << t2_start << endl;

cout << "flip type: "<<flipped << endl;

cout << "not flipped" << endl;
}

continue;
}
flipType = flipped;
}
else if (!isRectangleConflict(s1, s2, g1, g2, num_col,kDelay,abs(t1_start-t2_start),I_RM))
continue;
if (screen >= 5) {
cout << "is rectangle" << endl;
}

if (((s2/num_col - s1/num_col) * (s1 / num_col - g1 / num_col) < 0 && (s2 % num_col - s1 % num_col) * (s1% num_col - g1 % num_col) < 0)
|| ((s1 / num_col - s2 / num_col) * (s2 / num_col - g2 / num_col) < 0 && (s1% num_col - s2 % num_col) * (s2% num_col - g2 % num_col) < 0)) {
isChasing = true;
}


int new_type, new_area;
std::pair<int, int> Rg, Rs;

if (option.flippedRec) {
Rg = getFlippedRg(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(s2 / num_col, s2 % num_col),
                  std::make_pair(g1 / num_col, g1 % num_col),std::make_pair(g2 / num_col, g2 % num_col),flipType);
Rs = getFlippedRs(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(s2 / num_col, s2 % num_col),
                  std::make_pair(g1 / num_col, g1 % num_col), std::make_pair(g2 / num_col, g2 % num_col), flipType);
new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
bool kFullyBlocked = isKFullyBlocked(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(s2 / num_col, s2 % num_col),
                                     Rs, Rg, kDelay,t1_start,t2_start);
/*if (!kFullyBlocked) {
    if (screen >= 5) {
        cout << "not fully blocked" << endl;
    }
    continue;
}*/
new_type = classifyFlippedRectangleConflict(s1, s2, g1, g2, Rg, Rs, num_col, flipType, kFullyBlocked);
if (flipType == 1 && new_type <= -1) {
if (screen >= 5) {
cout << "fliptype 1 and non-cardinal" << endl;
}
continue;
}
if (flipType == 2 && new_type <= -1) {
if (screen >= 5) {
cout << "fliptype 2 and non-cardinal" << endl;
}
continue;
}
}
else {
Rg = getRg(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(g1 / num_col, g1 % num_col),
           std::make_pair(g2 / num_col, g2 % num_col));
Rs = getRs(std::make_pair(s1 / num_col, s1 % num_col), std::make_pair(s2 / num_col, s2 % num_col),
           std::make_pair(g1 / num_col, g1 % num_col));
new_area = (abs(Rs.first - Rg.first) + 1) * (abs(Rs.second - Rg.second) + 1);
new_type = classifyRectangleConflict(s1, s2, g1, g2, Rg, num_col,I_RM);


}
int new_distance = abs(s2 / num_col - s2 / num_col) + abs(s2 %num_col - s2 % num_col) +
                   abs(s1 / num_col - s1 / num_col) + abs(s1 %num_col - s1 % num_col);


if (new_type > type || (new_type == type && new_area > area) || (new_type == type && new_area == area && new_distance > distance))
{
if (screen >= 4) {
cout << "s1: " << s1 / num_col << " "<< s1 % num_col << endl;
cout << "g1: " << g1 / num_col << " "<< g1 % num_col << endl;
cout << "s1_t: " << t1_start << endl;

cout << "s2: " << s2 / num_col << " "<< s2 % num_col << endl;
cout << "g2: " << g2 / num_col << " "<< g2 % num_col << endl;
cout << "s2_t: " << t2_start << endl;

cout << "Rs: " << Rs.first << " " << Rs.second << endl;
cout << "Rg: " << Rg.first << " " << Rg.second << endl;

cout << "type " << new_type << " flip type " << flipType << endl;
}

auto new_rectangle = std::shared_ptr<Conflict>(new Conflict(loc1,con->k,timestep));
int Rs_t = con->t - abs(Rs.first - loc1 / num_col) - abs(Rs.second - loc1 % num_col);

int rt1, rt2;
if(option.RM4way>=2 && option.RM4way<=5) {
if (con->k == 0) {
rt1 = timestep -
      getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1 % num_col);
rt2 = rt1;
} else {
rt1 = timestep -
      getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1 % num_col);
rt2 = timestep2 -
      getMahattanDistance(Rs.first, Rs.second, loc1 / num_col, loc1 % num_col) -
      1;
}
}
else{
rt1=Rs_t;
rt2=Rs_t;
}
/*cout << "loc:" << loc1 << " t:" << timestep << endl;
cout << "s1 " << s1 << " s2 " << s2 << " g1 " << g1 << " g2 " << g2 << " rg " << Rg.first << " " << Rg.second <<" Rg_t "<< Rg_t<< endl;
*/								bool success;
//								if (option.flippedRec) {
//									success=new_rectangle->flippedRectangleConflict(a1, a2, Rs, Rg,
//										make_pair(s1 / num_col, s1 % num_col),
//										make_pair(s2 / num_col, s2 % num_col),
//										Rs_t, paths, t1_start, t2_start,
//										make_pair(g1 / num_col, g1 % num_col),
//										make_pair(g2 / num_col, g2 % num_col),
//										num_col, kDelay, flipType);
//
//								}
//								else{
success=new_rectangle->kRectangleConflict(a1, a2, Rs, Rg,
                                          make_pair(s1 / num_col, s1 % num_col),
                                          make_pair(s2 / num_col, s2 % num_col),
                                          rt1, rt2, paths, t1_start, t2_start,
                                          make_pair(g1 / num_col, g1 % num_col),
                                          make_pair(g2 / num_col, g2 % num_col),
                                          num_col, kDelay, option.RM4way,I_RM);
//								}

if (!success) {
if (screen >= 4)
cout << "Create rectangle conflict failed" << endl;
continue;
}

if (screen >= 4) {
cout << *new_rectangle << endl;
}


bool isBlocked = true;


for (auto constraint : new_rectangle->multiConstraint1) {
isBlocked = isBlocked && blocked(*paths[new_rectangle->a1], constraint);
}
for (auto constraint : new_rectangle->multiConstraint2) {
isBlocked = isBlocked && blocked(*paths[new_rectangle->a2], constraint);
}

if (option.RM4way>=3) {


if (isBlocked) {
previousRetangle[0] = a1;
previousRetangle[1] = a2;
previousRetangle[2] = Rs.first*num_col + Rs.second;
previousRetangle[3] = Rg.first*num_col + Rg.second;

//cout << "blocked, select this one" << endl;
rectangle = new_rectangle;
if (new_type == 2)
rectangle->p = conflict_priority::CARDINAL;
else if (new_type == 1) // && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::SEMI;
else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::NON;
type = new_type;
area = new_area;
distance = new_distance;
}
else {
if (screen >= 4)
cout << "4way RM not blocked" << endl;
}
}
else if (isBlocked)
{

previousRetangle[0] = a1;
previousRetangle[1] = a2;
previousRetangle[2] = Rs.first*num_col+Rs.second;
previousRetangle[3] = Rg.first*num_col+Rg.second;

//cout << "blocked, select this one" << endl;
rectangle = new_rectangle;
if (new_type == 2)
rectangle->p = conflict_priority::CARDINAL;
else if (new_type == 1) // && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::SEMI;
else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::NON;
type = new_type;
area = new_area;
distance = new_distance;
t1_startf= t1_start;
t1_endf= t1_end;
t2_startf= t2_start;
t2_endf= t2_end;
Rg_tf= Rs_t;
Rs_tf = Rs_t;
rt1_f = rt1;
rt2_f = rt2;

s1f= s1;
s2f= s2;
g1f= g1;
g2f= g2;
Rsf = Rs;
Rgf = Rg;

flipTypef = flipType;
if (I_RM)
isChasingf = isChasing;
}
else {
if (new_rectangle.unique()) {
new_rectangle.reset();
}
if (screen >= 4)
cout << "not blocked" << endl;
}

// rectangle = std::shared_ptr<tuple<int, int, int, int, int>>
//	(new tuple<int, int, int, int, int>(get<0>(*con), get<1>(*con), -1 - Rg.first * num_col - Rg.second, t1_start, t2_start));
}
}
}
}
}

if (type >= 0)
{

if (option.RM4way < 3) {
if (kDelay > 0) {
auto new_rectangle = std::shared_ptr<Conflict>(new Conflict(loc1, con->k, timestep));
/*if (kDelay > 0) {
    vector<MDDPath*> a1kMDD;
    vector<MDDPath*> a2kMDD;
    for (int i = 1; i <= kDelay; i++) {*/
MDD<Map> a1MDD;
MDD<Map> a2MDD;
updateConstraintTable(&parent, a1);
int tempk = kDelay;
//if (option.RM4way == 2) {
//	int tempk = kDelay > 1 ? 1 : kDelay;
//}

a1MDD.buildMDD(constraintTable, t1_endf - t1_startf + 1 + tempk,
*(search_engines[a1]), s1f, t1_startf, g1f,
paths[a1]->at(t1_startf).actionToHere);

updateConstraintTable(&parent, a2);
a2MDD.buildMDD(constraintTable, t2_endf - t2_startf + 1 + tempk,
*(search_engines[a2]), s2f, t2_startf, g2f,
paths[a2]->at(t2_startf).actionToHere);

MDDLevels a1MDDPath;
a1MDDPath = a1MDD.levels;

MDDLevels a2MDDPath;
a2MDDPath = a2MDD.levels;




//cout << "Create MDD Done" << endl;

//						if (option.flippedRec) {
//							new_rectangle->flippedRectangleConflict(a1, a2, Rsf, Rgf,
//								make_pair(s1f / num_col, s1f % num_col),
//								make_pair(s2f / num_col, s2f % num_col),
//								Rg_tf, paths, t1_startf, t2_startf,
//								make_pair(g1f / num_col, g1f % num_col),
//								make_pair(g2f / num_col, g2f % num_col),
//								num_col, kDelay, flipTypef, &a1MDDPath, &a2MDDPath);
//						}
//						else {
new_rectangle->kRectangleConflict(a1, a2, Rsf, Rgf,
        make_pair(s1f / num_col, s1f % num_col),
        make_pair(s2f / num_col, s2f % num_col),
        rt1_f, rt2_f, paths, t1_startf, t2_startf,
        make_pair(g1f / num_col, g1f % num_col),
        make_pair(g2f / num_col, g2f % num_col),
        num_col, kDelay, option.RM4way,I_RM, &a1MDDPath, &a2MDDPath);
//						}

rectangle = new_rectangle;
rectangle->isChasing = isChasingf;
}


if (type == 2)
rectangle->p = conflict_priority::CARDINAL;
else if (type == 1) // && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::SEMI;
else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::NON;
}




parent.conflicts.push_back(rectangle);
if (screen>=4)
cout << "add " << *rectangle << endl;

RMTime += std::clock() - RM_Start;

continue;

}
else {
RMTime += std::clock() - RM_Start;
}
}
else if (cons_strategy == constraint_strategy::CBSH_GR) // generalized rectangles
{
if (loc2 >= 0) // Edge conflict
continue;
if (paths[a1]->size() <= timestep || paths[a2]->size() <= timestep)//conflict happens after agent reaches its goal
continue;

int from1 = (*paths[a1])[timestep - 1].location;
int from2 = (*paths[a2])[timestep - 1].location;
if (from1 == from2 || // same direction
from1 == loc1 || from2 == loc1 || //wait actions
abs(from1 - from2) == 2 || abs(from1 - from2) == num_col * 2) //opposite direction
continue;

std::list<int>	s1s = getStartCandidates(*paths[a1], timestep, loc1 - from1, loc1 - from2);
std::list<int>	g1s = getGoalCandidates(*paths[a1], timestep, loc1 - from1, loc1 - from2);
std::list<int>	s2s = getStartCandidates(*paths[a2], timestep, loc1 - from1, loc1 - from2);
std::list<int>	g2s = getGoalCandidates(*paths[a2], timestep, loc1 - from1, loc1 - from2);


MDD<Map>* mdd1 = NULL, *mdd2 = NULL;
ConstraintsHasher c(a1, &parent,paths[a1]->size());
typename std::unordered_map<ConstraintsHasher, MDD<Map>*>::const_iterator got = mddTable[c.a].find(c);
if (got != mddTable[c.a].end())
{
mdd1 = got->second;
}
else
{
std::cout << "ERROR" << std::endl;
}
std::list<Constraint> B1;
bool haveBarriers = ExtractBarriers<Map>(*mdd1, loc1 - from1, loc1 - from2, paths[a1]->at(s1s.back()).location, paths[a1]->at(g1s.back()).location, s1s.back(), num_col, B1);
if (!haveBarriers)
continue;

c.a = a2;
got = mddTable[c.a].find(c);
if (got != mddTable[c.a].end())
{
mdd2 = got->second;
}
else
{
std::cout << "ERROR" << std::endl;
}
std::list<Constraint> B2;
haveBarriers = ExtractBarriers<Map>(*mdd2, loc1 - from2, loc1 - from1, paths[a2]->at(s2s.back()).location, paths[a2]->at(g2s.back()).location, s2s.back(), num_col, B2);
if (!haveBarriers)
continue;

// Try all possible combinations
int type = -1;
list<Constraint>::const_iterator b1_entry = B1.cbegin();
list<Constraint>::const_iterator b2_entry = B2.cbegin();
std::pair<int, int> Rs, Rg;
std::set<std::pair<int, int>> visitedRs;
generalizedRectangle<Map>(*paths[a1], *paths[a2], *mdd1, *mdd2, b1_entry, b2_entry, B1, B2, timestep, num_col, type, Rs, Rg, std::clock() * CLOCKS_PER_SEC * 1.0 / 1000 + time_limit - runtime, visitedRs);
if (type < 0)
continue;
int Rg_t = timestep + abs(Rg.first - loc1 / num_col) + abs(Rg.second - loc1 % num_col);
std::shared_ptr<Conflict> rectangle = std::shared_ptr<Conflict>(new Conflict());
rectangle->rectangleConflict(a1, a2, Rs, Rg, loc1 - from1, loc1 - from2, Rg_t, paths, num_col);
if (type == 2)
rectangle->p = conflict_priority::CARDINAL;
else if (type == 1) // && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::SEMI;
else //if (type == 0 && !findRectangleConflict(parent.parent, *conflict))
rectangle->p = conflict_priority::NON;
if (blocked(*paths[rectangle->a1], rectangle->constraint1) && blocked(*paths[rectangle->a2], rectangle->constraint2))
{
parent.conflicts.push_back(rectangle);
continue;
}
}