#include <iostream>
using namespace std;

#include "GameStateManager.h"
#include "instructionstate.h"
#include "menustate.h"

CInstructionState CInstructionState::theInstructionState;

void CInstructionState::Init()
{
	theScene = new CSceneManager2D(800, 600);
	theScene->Init();
}

void CInstructionState::Init(const int width, const int height)
{
	theScene = new CSceneManager2D(800, 600);
	theScene->Init();
}

void CInstructionState::Cleanup()
{
	theScene->Exit();
	delete theScene;
	theScene = NULL;
}

void CInstructionState::Pause()
{

}

void CInstructionState::Resume()
{

}

void CInstructionState::HandleEvents(CGameStateManager* theGSM)
{}

void CInstructionState::HandleEvents(CGameStateManager* theGSM, const unsigned char key, const bool status)
{}

void CInstructionState::HandleEvents(CGameStateManager* theGSM, const double mouse_x, const double mouse_y,
	const int button_Left, const int button_Middle, const int button_Right)
{}

void CInstructionState::Update(CGameStateManager* theGSM)
{
}

void CInstructionState::Update(CGameStateManager* theGSM, const double m_dElapsedTime)
{
	if (Application::IsKeyPressed(VK_BACK))
	{
		theGSM->ChangeState(CMenuState::Instance());
	}
}

void CInstructionState::Draw(CGameStateManager* theGSM)
{
	theScene->RenderInstructions();
}
