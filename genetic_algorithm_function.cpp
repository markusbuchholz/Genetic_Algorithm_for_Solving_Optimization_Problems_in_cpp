// Markus Buchholz
#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>

//--------Path Planner--------------------------------------------------------------

float xmin = -5.0; // 0.0;
float xmax = 5.0;  // 50.0;
float ymin = -5.0; // 0.0;
float ymax = 5.0;  // 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//--------------------------------------------------------------------------------
int DIM = 2;
int EVOLUTIONS = 1000;
int CHROMOSOMES = 100;

int ELITISM = 20;   // 0.1 * CHROMOSOMES;
int CROSSOVER = 50; // 0.5 * CHROMOSOMES;
int MUTATION = 30;  // 0.4 * CHROMOSOMES;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
};

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

float generateRandomX()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);
    return distribution(gen);
}
//--------------------------------------------------------------------------------

float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {

        funcValue.push_back(ii.x * ii.y);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    return pos.x * pos.y;
}

//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }

    return Pnew;
}
//--------------------------------------------------------------------------------
int dimensionToUpdate()
{

    std::random_device engine;
    std::mt19937 gen(engine());
    std::uniform_int_distribution<int> distribution(0, DIM - 1);
    return distribution(gen);
}

//--------------------------------------------------------------------------------

Pos posMutation(Pos chromA, Pos chromB)
{

    int dominant = dimensionToUpdate();
    Pos Xnew;

    if (dominant == 0)
    {

        Xnew.x = chromA.x + generateRandomX() * (chromA.x - chromB.x);
        Xnew.y = chromA.y + generateRandomX() * (chromA.y - chromB.y);
    }
    else if (dominant == 1)
    {
        Xnew.x = chromB.x + generateRandomX() * (chromB.x - chromA.x);
        Xnew.y = chromB.y + generateRandomX() * (chromB.y - chromA.y);
    }

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

Pos posCrossover(Pos chromA, Pos chromB)
{

    int cross = dimensionToUpdate();
    Pos Xnew;

    if (cross == 0)
    {
        Xnew.x = chromA.x;
        Xnew.y = chromB.y;
    }
    else if (cross == 1)
    {
        Xnew.x = chromB.x;
        Xnew.y = chromA.y;
    }

    return positionUpdateCheck(Xnew);
}

//--------------------------------------------------------------------------------

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < CHROMOSOMES; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
    }

    return pos;
}

//--------------------------------------------------------------------------------

Pos newPosXY()
{

    Pos pos = {valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)};

    // return positionUpdateCheck(pos);
    return pos;
}

//-------------------------------------------------------------------------------
bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}

//-------------------------------------------------------------------------------

// min
std::tuple<std::vector<Pos>, std::vector<Pos>, std::vector<Pos>> evolutionSet(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;
    std::vector<Pos> rest;

    std::vector<Pos> elit;
    std::vector<Pos> cross;
    std::vector<Pos> mutation;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMin);

    for (int ii = 0; ii < best.size(); ii++)
    {
        auto pos = std::get<0>(best[ii]);
        if (ii < ELITISM)
        {

            elit.push_back(pos);
        }
        else if (ii > ELITISM)
        {
            rest.push_back(pos);
        }
    }

    // std::random_shuffle(rest.begin(), rest.end());

    for (int ii = 0; ii < rest.size(); ii++)
    {

        if (ii < CROSSOVER)
        {
            cross.push_back(rest[ii]);
        }
        else if (ii > CROSSOVER)
        {
            mutation.push_back(rest[ii]);
        }
    }
    return std::make_tuple(elit, cross, mutation);
}

//-------------------------------------------------------------------------------

int chooseChromosome(int actual)
{

    std::random_device engine;
    std::uniform_int_distribution<int> distribution(0, CHROMOSOMES);

    int r = -1;

    do
    {

        r = distribution(engine);

    } while (r == actual);

    return r;
}

//-------------------------------------------------------------------------------

void runGA()
{

    std::vector<Pos> currentPositions = initPosXY();
    std::vector<float> valueFunction;

    for (int ii = 0; ii < EVOLUTIONS; ii++)
    {

        std::vector<float> currentValueFunction = function(currentPositions);
        auto evoSet = evolutionSet(currentPositions, currentValueFunction);
        currentPositions.erase(currentPositions.begin(), currentPositions.end());

        std::vector<Pos> elite = std::get<0>(evoSet);
        std::vector<Pos> cross = std::get<1>(evoSet);
        std::vector<Pos> mutation = std::get<2>(evoSet);

        std::vector<Pos> evoPositions(elite);

        for (int jj = 0; jj < cross.size(); jj++)
        {

            int chromB_pos = chooseChromosome(jj);
            Pos newCross = posCrossover(cross[jj], cross[chromB_pos]);
            evoPositions.push_back(newCross);
        }

        for (int kk = 0; kk < mutation.size(); kk++)
        {

            int chromB_pos = chooseChromosome(kk);
            Pos newMutation = posMutation(mutation[kk], mutation[chromB_pos]);
            evoPositions.push_back(newMutation);
        }

        currentPositions = evoPositions;
        // for (auto &ii : evoPositions)
        // {
        //     std::cout << ii.x << " , " << ii.y << "\n";
        // }

        valueFunction = currentValueFunction;
    }

    for (auto &ii : valueFunction)
    {
        std::cout << ii << "\n";
    }
}

//-------------------------------------------------------------------------------

int main()
{

    runGA();
}
