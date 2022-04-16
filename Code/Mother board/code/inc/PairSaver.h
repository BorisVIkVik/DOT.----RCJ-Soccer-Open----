#pragma once

#include <stdint.h>
#include <queue>
#include <utility>

using namespace std;

class PairSaver
{
    public:
        PairSaver(){}
        void add(pair<double, double> speed, uint32_t time);
        pair<double, double> pop(uint32_t time);
		
    private:
        queue<pair<pair<double, double>, uint32_t> > pairArr;
};

void	PairSaver::add(pair<double, double> speed, uint32_t time)
{
    pairArr.push(make_pair(make_pair(speed.first, speed.second), time));
}

pair<double, double>	PairSaver::pop(uint32_t time)
{
    while(pairArr.front().second < time)
        pairArr.pop();
    return pairArr.front().first;
}
