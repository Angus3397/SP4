#include <iostream>
using namespace std;

#include "GameStateManager.h"
#include "menustate.h"
#include "highscorestate.h"

CHighscoreState CHighscoreState::theHighscoreState;

void CHighscoreState::Init()
{
	theScene = new CSceneManager2D(800, 600);
	theScene->Init();
}

void CHighscoreState::Init(const int width, const int height)
{
	theScene = new CSceneManager2D(width, height);
	theScene->Init();
}

void CHighscoreState::Cleanup()
{
	theScene->Exit();
	delete theScene;
	theScene = NULL;
}

void CHighscoreState::Pause()
{
}

void CHighscoreState::Resume()
{
}

void CHighscoreState::HandleEvents(CGameStateManager* theGSM)
{
}

void CHighscoreState::HandleEvents(CGameStateManager* theGSM, const unsigned char key, const bool status)
{
}

void CHighscoreState::HandleEvents(CGameStateManager* theGSM, const double mouse_x, const double mouse_y,
	const int button_Left, const int button_Middle, const int button_Right)
{
}

void CHighscoreState::Update(CGameStateManager* theGSM)
{
}

void CHighscoreState::Update(CGameStateManager* theGSM, const double m_dElapsedTime)
{
	if (Application::IsKeyPressed(VK_BACK))
	{
		theGSM->ChangeState(CMenuState::Instance());
	}
}

void CHighscoreState::Draw(CGameStateManager* theGSM)
{
	theScene->RenderHighscore();
}
