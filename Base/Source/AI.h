#ifndef AI_H
#define AI_H


#include <vector>

class cAI
{
	enum FSM_ENEMY
	{
		STATE_ATTACK,
		STATE_CHASE,
		STATE_PATROL,
	};

public:
	cAI();
	virtual ~cAI();

	void init();
	void update(double dt);

	FSM_ENEMY getState();
	//void setState(FSM_ENEMY state);

	FSM_ENEMY FSM;
};
#endif