#pragma once

#include <stdint.h>
#include <queue>
#include <utility>

using namespace std;

class SpeedSaver
{
    public:
        SpeedSaver();
        void add(pair<double, double> speed, uint32_t time);
        pair<double, double> pop(uint32_t time);
    private:
        queue<pair<pair<double, double>, uint32_t> > spArr;
};

void	SpeedSaver::add(pair<double, double> speed, uint32_t time)
{
    spArr.push(make_pair(make_pair(speed.first, speed.second), time));
}

pair<double, double>	SpeedSaver::pop(uint32_t time)
{
    while(spArr.front().second < time)
        spArr.pop();
    return spArr.front().first;
}