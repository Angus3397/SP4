#include "Player.h"


Player::Player(void) 
: thePlayerinfoLua(NULL)
, AmtOfClearedLevelEasy(0)
, AmtOfClearedLevelNormal(0)
, AmtOfClearedLevelHard(0)
{

}

Player::~Player(void)
{
	if (thePlayerinfoLua)
	{
		delete thePlayerinfoLua;
		thePlayerinfoLua = NULL;
	}
}

void Player::PlayerInit(string PlayerFileName)
{
	thePlayerinfoLua = new LuaUsage();
	thePlayerinfoLua->LuaUsageInit(PlayerFileName);
	this->AmtOfClearedLevelEasy = thePlayerinfoLua->GetIntegerValue("EASYLEVELSCLEARED");
	this->AmtOfClearedLevelNormal = thePlayerinfoLua->GetIntegerValue("NORMALLEVELSCLEARED");
	this->AmtOfClearedLevelHard = thePlayerinfoLua->GetIntegerValue("HARDLEVELSCLEARED");
	thePlayerinfoLua->LuaUsageClose();
}

void Player::SetAmtOfClearedLevelEasy(short AmtOfClearedLevelEasy)
{
	this->AmtOfClearedLevelEasy = AmtOfClearedLevelEasy;
}

short Player::GetAmtOfClearedLevelEasy()
{
	return this->AmtOfClearedLevelEasy;
}
void Player::SetAmtOfClearedLevelNormal(short AmtOfClearedLevelNormal)
{
	this->AmtOfClearedLevelNormal = AmtOfClearedLevelNormal;
}
short Player::GetAmtOfClearedLevelNormal()
{
	return this->AmtOfClearedLevelNormal;
}
void Player::SetAmtOfClearedLevelHard(short AmtOfClearedLevelHard)
{
	this->AmtOfClearedLevelHard = AmtOfClearedLevelHard;
}
short Player::GetAmtOfClearedLevelHard()
{
	return this->AmtOfClearedLevelHard;
}