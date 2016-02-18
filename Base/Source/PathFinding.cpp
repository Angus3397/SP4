#include "PathFinding.h"

CPathFinding::CPathFinding(void)
{
	m_initialisedStartGoal = false;
	m_foundGoal = false;
}

CPathFinding::~CPathFinding(void)
{
}

void CPathFinding::FindPath(Vector3 currentPos, Vector3 targetPos)
{
	if (!m_initialisedStartGoal)
	{
		for (int i = 0; i < m_openList.size(); i++)
		{
			delete m_openList[i];
		}
		m_openList.clear();

		for (int i = 0; i < m_visitedList.size(); i++)
		{
			delete m_visitedList[i];
		}
		m_visitedList.clear();

		for (int i = 0; i < m_pathToGoal.size(); i++)
		{
			delete m_pathToGoal[i];
		}
		m_pathToGoal.clear();

		// Initialise start
		CSearchCell start;
		start.m_xCoord = currentPos.x;
		start.m_zCoord = currentPos.z;

		// Initialise goal
		CSearchCell goal;
		goal.m_xCoord = targetPos.x;
		goal.m_zCoord = targetPos.z;

		SetStartAndGoal(start, goal);
		m_initialisedStartGoal = true;
	}
	if (m_initialisedStartGoal)
	{
		ContinuePath();
	}
}

void CPathFinding::SetStartAndGoal(CSearchCell start, CSearchCell goal)
{
	m_startCell = new CSearchCell(start.m_xCoord, start.m_zCoord, NULL);
	m_goalCell = new CSearchCell(goal.m_xCoord, goal.m_zCoord, &goal);

	m_startCell->G = 0;
	m_startCell->H = m_startCell->ManhattanDist(m_goalCell);
	m_startCell->parent = 0;

	m_openList.push_back(m_startCell);
}

CSearchCell* CPathFinding::GetNextCell()
{
	float bestF = 999999.0f;
	int cellIndex = -1;
	CSearchCell* nextCell = NULL;

	for (int i = 0; i < m_openList.size(); i++)
	{
		if (m_openList[i]->GetF() < bestF)
		{
			bestF = m_openList[i]->GetF();
			cellIndex = i;
		}
	}

	if (cellIndex >= 0)
	{
		nextCell = m_openList[cellIndex];
		m_visitedList.push_back(nextCell);
		m_openList.erase(m_openList.begin() + cellIndex);
	}

	return nextCell;
}