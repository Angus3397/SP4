#include "Save.h"

Save::Save(void)
{

}

Save::~Save(void)
{
}

void Save::SavePlayer(Player* playerinfo)
{
	ofstream SaveFile("Player.lua");
	if (SaveFile.is_open())
	{
		SaveFile << "--Player Save File--" << endl;
		SaveFile << "EASYLEVELSCLEARED = " << playerinfo->GetAmtOfClearedLevelEasy() << endl;
		SaveFile << "NORMALLEVELSCLEARED = " << playerinfo->GetAmtOfClearedLevelNormal() << endl;
		SaveFile << "HARDLEVELSCLEARED = " << playerinfo->GetAmtOfClearedLevelHard() << endl;
		SaveFile.close();
	}
}
