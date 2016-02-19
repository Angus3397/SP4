#ifndef SAVE_H
#define SAVE_H

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include "Player.h"

using namespace std;

class Save
{
public:
	Save(void);
	~Save(void);

	void SavePlayer(Player* playerinfo);
	//void SaveLevel(/*Level* Levelinfo*/);
private:
};
#endif