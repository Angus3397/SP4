#ifndef SOUND_MANAGER_H
#define SOUND_MANAGER_H

#include <iostream>
#include <irrKlang.h>
#pragma comment(lib, "irrKlang.lib")

using namespace irrklang;	// Link with irrklang.dll

class CSoundManager
{

public:
	ISoundEngine* engine;

	CSoundManager(void);
	~CSoundManager(void);

	void adjustVol();

	//Main Menu
	void mainMenuBGM();
	void ConfirmSound();
	void SelectSound();

	//Level BGMs
	void level1BGM();
	void level2BGM();
	void level3BGM();
	void level4BGM();

	//Player interactions


	//Others
	float volume;

	bool clicked;
};

#endif

