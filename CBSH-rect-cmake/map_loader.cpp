#include "map_loader.h"
#include <iostream>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <cstring>
#include <bitset>



using namespace boost;
using namespace std;


MapLoader::MapLoader(){}



vector<pair<int, int>> MapLoader::get_transitions(int loc, int heading, int noWait) const {
    vector<pair<int, int>> transitions;
    int moveRange = 5;

    for (int direction = 0; direction < moveRange; direction++)
    {
        pair<int, int> move;
        int next_loc = loc + moves_offset[direction];
        move.first = next_loc;
        move.second = -1; //-1 means no heading
        if (validMove(loc, next_loc) && !my_map[next_loc])
        {
            transitions.push_back(move);

        }

    }

    return transitions;

}

bool MapLoader::getLoc(int loc)  {
	return my_map[loc];
}
int MapLoader::getDegree(int loc)
{
	if (loc < 0 || loc >= map_size() || getLoc(loc))
		return -1;
	int degree = 0;
	if (0 < loc - cols && !getLoc(loc - cols))
		degree++;
	if (loc + cols < map_size() && !getLoc(loc + cols))
		degree++;
	if (loc % cols > 0 && !getLoc(loc - 1))
		degree++;
	if (loc % cols < cols - 1 && !getLoc(loc + 1))
		degree++;
	return degree;
}

bool MapLoader::isFullyBlocked(int start, int end) {
	int start_x = start / cols;
	int start_y = start % cols;
	int end_x = end / cols;
	int end_y = end % cols;

	int sign_x = start_x < end_x ? 1 : -1;
	int sign_y = start_y < end_y ? 1 : -1;
	for (int x = start_x; x - sign_x != end_x; x = x + sign_x) {
		for (int y = start_y; y - sign_y != end_y; y = y + sign_y) {
			if (my_map[linearize_coordinate(x, y)])
				return false;
		}
	}
	return true;
}


bool MapLoader::validMove(int curr, int next) const
{
	if (next < 0 || next >= rows * cols)
		return false;
	int curr_x = curr / cols;
	int curr_y = curr % cols;
	int next_x = next / cols;
	int next_y = next % cols;
	return abs(next_x - curr_x) + abs(next_y - curr_y) < 2;
}


MapLoader::MapLoader(int rows, int cols)
{
  int i, j;
  this->rows = rows;
  this->cols = cols;
  this->my_map = new bool[rows*cols];
  for (i=0; i<rows*cols; i++)
    this->my_map[i] = false;
  // Possible moves [WAIT, NORTH, EAST, SOUTH, WEST]
  moves_offset = new int[MapLoader::MOVE_COUNT];
  moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
  moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
  moves_offset[MapLoader::valid_moves_t::EAST] = 1;
  moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
  moves_offset[MapLoader::valid_moves_t::WEST] = -1;

  // add padding
  i = 0;
  for (j=0; j<cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  i=rows-1;
  for (j=0; j<cols; j++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j=0;
  for (i=0; i<rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;
  j=cols-1;
  for (i=0; i<rows; i++)
    this->my_map[linearize_coordinate(i,j)] = true;

}

MapLoader::MapLoader(string fname)
{
  string line;
  ifstream myfile (fname.c_str());
  if (myfile.is_open()) {
    getline (myfile,line);
    char_separator<char> sep(",");
    tokenizer< char_separator<char> > tok(line, sep);
    tokenizer< char_separator<char> >::iterator beg=tok.begin();
    int rows = atoi ( (*beg).c_str() ); // read number of rows
    beg++;
    int cols = atoi ( (*beg).c_str() ); // read number of cols
    bool* my_map= new bool[rows*cols];
    for (int i=0; i<rows*cols; i++)
      my_map[i] = false;
    // read map (and start/goal locations)
    for (int i=0; i<rows; i++) {
		getline (myfile, line);
		for (int j=0; j<cols; j++) {
		  my_map[cols*i + j] = (line[j] != '.');
		}
    }

    myfile.close();
    this->rows = rows;
    this->cols = cols;
    this->my_map = my_map;
    // initialize moves_offset array
    moves_offset = new int[MapLoader::MOVE_COUNT];
    moves_offset[MapLoader::valid_moves_t::WAIT_MOVE] = 0;
    moves_offset[MapLoader::valid_moves_t::NORTH] = -cols;
    moves_offset[MapLoader::valid_moves_t::EAST] = 1;
    moves_offset[MapLoader::valid_moves_t::SOUTH] = cols;
    moves_offset[MapLoader::valid_moves_t::WEST] = -1;
  }
  else
  {
	cerr << "Map file " << fname << " not found." << std::endl;
	exit(10);
	}
}


char* MapLoader::mapToChar() 
{
  char* mapChar = new char[rows*cols];
  for (int i=0; i<rows*cols; i++) {
    if ( i == start_loc )
      mapChar[i] = 'S';
    else if ( i == goal_loc )
      mapChar[i] = 'G';
    else if (this->my_map[i] == true)
      mapChar[i] = '*';
    else
      mapChar[i] = ' ';
  }
  return mapChar;
}

void MapLoader::printMap () 
{
  char* mapChar = mapToChar();
  printMap (mapChar);
  delete[] mapChar;
}


void MapLoader::printMap (char* mapChar) 
{
  cout << "MAP:";
  for (int i=0; i<rows*cols; i++) {
    if (i % cols == 0)
      cout << endl;
    cout << mapChar[i];
  }
  cout << endl;
}


bool* MapLoader::get_map() const {
  bool* retVal = new bool [ this->rows * this->cols ];
  memcpy (retVal, this->my_map, sizeof(bool)* this->rows * this->cols );
  return retVal;
}

MapLoader::~MapLoader() {
  //delete[] this->my_map;
  delete[] this->moves_offset;
}

void MapLoader::saveToFile(std::string fname) 
{
	ofstream myfile;
	myfile.open (fname);
	myfile << rows << "," << cols << endl;
	for (int i=0; i<rows; i++) 
	{
		for (int j=0; j<cols; j++) 
		{
			if ( my_map[linearize_coordinate(i,j)] == true)
				myfile << "@";
			else
				myfile << ".";
		}
		myfile << endl;
	}
	myfile.close();
}

void MapLoader::printPath(vector<int> path)
{
	  for (size_t i=0; i<path.size(); i++) 
			cout << "[" << row_coordinate(path[i]) << "," << col_coordinate(path[i]) << "] ; ";
	  cout << endl;
}

