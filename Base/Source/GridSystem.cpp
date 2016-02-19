#include "GridSystem.h"
#include <iostream>

GridSystem::GridSystem()
{
}


GridSystem::~GridSystem()
{
	if (GridsVec.size() > 0)
	{
		for (int a = 0; a < GridsVec.size(); a++)
		{
			delete GridsVec[a];
		}
		GridsVec.clear();
	}
}

void GridSystem::Init(Vector3 Pos, float LengthX, float LengthY, int NumOfGridsX, int NumOfGridsY)
{
	this->Pos = Pos;
	this->LengthOfGridsX = LengthX;
	this->LengthOfGridsY = LengthY;
	this->NumOfGridsX = NumOfGridsX;
	this->NumOfGridsY = NumOfGridsY;
	
	
	int checkOddGridsX = NumOfGridsX % 2;
	int checkOddGridsY = NumOfGridsY % 2;

	float x = 0.f;
	float y = 0.f;

	float offsetX = 0.f;
	float offsetY = 0.f;

	for (int a = NumOfGridsY; a > 0; a--)
	{
		for (int b = 0; b < NumOfGridsX; b++)
		{
			//calculation of x pos for grids
			if (checkOddGridsX)
			{
				int mid = (NumOfGridsX + 1) * 0.5;
				offsetX = LengthX * (mid - (b + 1));
			}
			else
			{
				int mid = NumOfGridsX * 0.5;
				offsetX = LengthX * (mid - (b + 1)) + LengthX * 0.5;
			}
			x = this->Pos.x - offsetX;

			//calculation of y pos for grids
			if (checkOddGridsY)
			{
				int mid = (NumOfGridsY + 1) * 0.5;
				offsetY = LengthY * (mid - a);
			}
			else
			{
				int mid = NumOfGridsY * 0.5;
				offsetY = LengthY * (mid - a) + LengthY * 0.5;
			}
			y = this->Pos.y - offsetY;

			//set x and y as grid pos and push into vector
			Grid* PlayGrid = new Grid();
			PlayGrid->SetPos(x, y);
			GridsVec.push_back(PlayGrid);
			std:: cout << x << ", " << y << std::endl;
		}
	}
	
}
void GridSystem::UpdateGrid(Vector3 CursorPos)
{
	for (int a = 0; a < GridsVec.size(); a++)
	{
		Vector3 GridPos = GridsVec[a]->GetPos();
		// detection is due to reference point being at the bottom left
		if (GridPos.x < CursorPos.x && CursorPos.x < GridPos.x + LengthOfGridsX)
		{
			if (GridPos.y < CursorPos.y && CursorPos.y < GridPos.y + LengthOfGridsY)
			{
				GridsVec[a]->ChangeType();
			//	std::cout << GridsVec[a]->GetPos() << std::endl;
			}
		}
	}
}

void GridSystem::SetAnswer()
{

}

vector<Grid*> GridSystem::GetGridsVec()
{
	return this->GridsVec;
}