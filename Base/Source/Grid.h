#pragma once
#include "Vector3.h"

class Grid
{
public:
	enum GridType
	{
		EMPTY = 0,
		CROSS,
		FILLED,
		TOTAL_TYPE,
	};

	Grid();
	~Grid();

	int GetType(void);
	void ChangeType(void);

	void SetPos(float, float);
	Vector3 GetPos();

private:
	GridType Type;
	Vector3 Pos;
};

