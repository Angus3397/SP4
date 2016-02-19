#include <iostream>
using namespace std;

#include "GameStateManager.h"
#include "optionstate.h"
#include "menustate.h"

COptionState COptionState::theOptionState;

void COptionState::Init()
{
	theScene = new CSceneManager2D(800, 600);
	theScene->Init();
}

void COptionState::Init(const int width, const int height)
{
	theScene = new CSceneManager2D(800, 600);
	theScene->Init();
}

void COptionState::Cleanup()
{
	theScene->Exit();
	delete theScene;
	theScene = NULL;
}

void COptionState::Pause()
{

}

void COptionState::Resume()
{

}

void COptionState::HandleEvents(CGameStateManager* theGSM)
{}

void COptionState::HandleEvents(CGameStateManager* theGSM, const unsigned char key, const bool status)
{}

void COptionState::HandleEvents(CGameStateManager* theGSM, const double mouse_x, const double mouse_y,
	const int button_Left, const int button_Middle, const int button_Right)
{}

void COptionState::Update(CGameStateManager* theGSM)
{
}

void COptionState::Update(CGameStateManager* theGSM, const double m_dElapsedTime)
{
	if (Application::IsKeyPressed(VK_BACK))
	{
		theGSM->ChangeState(CMenuState::Instance());
	}
}

void COptionState::Draw(CGameStateManager* theGSM)
{
	theScene->RenderOption();
}
