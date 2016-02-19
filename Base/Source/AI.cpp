#include "AI.h"

using namespace std;

cAI::cAI()
{
}

cAI::~cAI()
{
}

void cAI::init()
{
	FSM = STATE_PATROL;
}

cAI::FSM_ENEMY cAI::getState()
{
	return FSM;
}

void cAI::update(double dt)
{
	switch (FSM)
	{
		case STATE_PATROL:
		{
			//if (detected)
			FSM = STATE_CHASE;
			break;
		}

		case STATE_CHASE:
		{
			//if (AI.pos == player.pos)
			FSM = STATE_ATTACK;
			//else if(!detected)
			FSM = STATE_PATROL;
			break;
		}

		case STATE_ATTACK:
		{
			//player.setHealth(player.getHealth() - AI.getDamage());
			//remove enemy
			break;
		}
		default:
			break;
	}
}