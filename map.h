#ifndef MAP_H_
#define MAP_H_

#include <vector>
#include "geometry.h"
using namespace std;
using namespace geometry;

class Map
{
public:
	char raw_data[MAP_SIZE][MAP_SIZE];
	int index;
	void setIndex(int index_) { index = index_; }
	int getIndex() { return index; }
private:
	//int index;
};



#endif // !MAP_H_


