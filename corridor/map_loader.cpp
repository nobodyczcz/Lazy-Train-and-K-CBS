#include "map_loader.h"
#include <iostream>
#include <fstream>
#include<boost/tokenizer.hpp>
#include <memory.h>



using namespace boost;
using namespace std;

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
  if (myfile.is_open()) 
  {
	  tokenizer< char_separator<char> >::iterator beg;
    getline (myfile,line);
	if (line[0] == 't') // Nathan's benchmark
	{
		char_separator<char> sep(" ");
		getline(myfile, line);
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		beg++;
		rows = atoi((*beg).c_str()); // read number of rows
		getline(myfile, line);
		tokenizer< char_separator<char> > tok2(line, sep);
		beg = tok2.begin();
		beg++;
		cols = atoi((*beg).c_str()); // read number of cols
		getline(myfile, line); // skip "map"
	}
	else // my benchmark
	{
		char_separator<char> sep(",");
		tokenizer< char_separator<char> > tok(line, sep);
		beg = tok.begin();
		rows = atoi((*beg).c_str()); // read number of rows
		beg++;
		cols = atoi((*beg).c_str()); // read number of cols
	}
    my_map= new bool[rows*cols];
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
  delete[] this->my_map;
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

