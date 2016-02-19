#ifndef PLAYER_H
#define PLAYER_H

#include <iostream>
#include <string>
#include "LuaUsage.h"

using namespace std;

class Player
{
public:
	Player(void);
	~Player(void);

	void PlayerInit(string PlayerFileName);

	void SetAmtOfClearedLevelEasy(short AmtOfClearedLevelEasy);
	short GetAmtOfClearedLevelEasy();
	void SetAmtOfClearedLevelNormal(short AmtOfClearedLevelNormal);
	short GetAmtOfClearedLevelNormal();
	void SetAmtOfClearedLevelHard(short AmtOfClearedLevelHard);
	short GetAmtOfClearedLevelHard();

private:
	short AmtOfClearedLevelEasy;
	short AmtOfClearedLevelNormal;
	short AmtOfClearedLevelHard;
	LuaUsage* thePlayerinfoLua;
};
#endif