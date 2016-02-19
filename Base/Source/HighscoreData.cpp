#include "HighscoreData.h"


HighscoreData::HighscoreData()
	: MAX_SCORES(5)
	, newScore(-1)
	, counter(0)
{
}

HighscoreData::~HighscoreData()
{
}

int HighscoreData::ReadTextFile(string filename)
{
	ifstream is(filename);
	int i = 0;

	if (is.is_open()) // if the file is open
	{
		while (!is.eof()) // while it is not the end of the file
		{
			int temp; // make a temporary variable
			is >> temp; // write the value into this temporary value.
			this->scores[i].addScore(temp); // add the temp value into the highscore database
			
			i++; // increment the i
			counter++; // increment the counter
		}
		is.close();
	}
	return i;
}

int HighscoreData::WriteTextFile(string filename)
{
	ofstream of(filename);
	int i = 0;
	if (of.is_open()) // if the file is open
	{
		while (i < MAX_SCORES - 1) 
		{
			of << this->scores[i].getScore(); // write the score into the text file
			if (i < MAX_SCORES - 1)
			{
				of << endl;
			}
			i++; // increment the i
			counter--; // decrement the counter
		}
	}
	return i;
}

void HighscoreData::AddScores(Highscore theHighscore)
{
	if (counter < MAX_SCORES) // if the counter is not at it's limit
	{
		this->scores[counter] = theHighscore;
		counter++; // increment counter
		MergeSortHighscore(this->scores, 0, counter - 1); // sort
	}
	else
	{
		this->scores[MAX_SCORES - 1] = theHighscore;
		MergeSortHighscore(this->scores, 0, counter - 1); // sort
	}
}


void HighscoreData::MergeHighscore(Highscore list[], int first, int middle, int last)
{
	Highscore* temp = new Highscore[middle - first + 1];
	int i, j, k; // i - temp array, j - 2nd list, k - combine list

	for (j = first, i = 0; j <= middle; i++, j++)
	{
		temp[i] = list[j]; // duplicate the first list(list[]).
	}

	i = 0;
	k = first;
	while (k < j && j <= last)
	{
		if (temp[i].getScore() >= list[j].getScore()) // if the score in i is bigger than the score in j
			list[k++] = temp[i++]; // put i into k
		else // if the score in i is smaller than the score in j
			list[k++] = temp[j++]; // put j into k
	}
	while (k < j)
	{
		list[k++] = temp[i++];
	}

	delete[] temp; // remove the temp array
}

void HighscoreData::MergeSortHighscore(Highscore list[], int first, int last)
{
	if (first < last)
	{
		int middle = (first + last) / 2;
		MergeSortHighscore(list, first, middle);
		MergeSortHighscore(list, middle + 1, last);
		MergeHighscore(list, first, middle, last);
	}
}

int HighscoreData::GetAllHighscores(int index)
{
	return scores[index].getScore();
}