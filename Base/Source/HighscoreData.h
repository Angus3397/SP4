#pragma once
#include "Highscore.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

class HighscoreData
{
private:
	int counter;
	int newScore;
	int MAX_SCORES;
	Highscore scores[5]; // Make the array of scores, 5 is the limit

public:
	HighscoreData();
	~HighscoreData();

	int ReadTextFile(string filename); // read from text file
	int WriteTextFile(string filename); // write to text file

	void AddScores(Highscore theHighscore);
	void UpdateHighscore(Highscore theHighscore);

	void MergeHighscore(Highscore list[], int first, int middle, int last);
	void MergeSortHighscore(Highscore list[], int first, int last);

	int GetAllHighscores(int index);
};

