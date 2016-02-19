#ifndef LUAUSAGE_H
#define LUAUSAGE_H

#include <iostream>
#include <string>
#include <fstream>
#include "lua.hpp"

using namespace std;

class LuaUsage
{
public:
	LuaUsage(void);
	~LuaUsage(void);

	void LuaUsageInit(string LuaFileName);
	void LuaUsageClose();

	int GetIntegerValue(string Values);
	bool GetBooleanValue(string Values);
	string GetStringValue(string Values);
	char GetCharacterValue(string Values);

	void LuaErrorLog(string Errorstring);

private:
	lua_State *LuaState;
};
#endif