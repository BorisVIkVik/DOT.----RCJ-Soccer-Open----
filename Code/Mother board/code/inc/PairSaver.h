#pragma once

#include <stdint.h>
#include <queue>
#include <utility>
#include <vector>

using namespace std;


#define SAVER_BUFFER_SIZE 20


class PairSaver
{
    public:
				PairSaver()
				{
					bufStart = 0;
					bufEnd = 0;
					buffer[bufStart] = make_pair(make_pair(0, 0), 0);
				}
        void add(pair<double, double> val, uint32_t time);
        pair<double, double> pop(uint32_t time);
		
    private:
				void incP(uint8_t &p);
        pair<pair<double,double>, uint32_t> buffer[SAVER_BUFFER_SIZE];
				uint8_t bufStart;
				uint8_t bufEnd;
};

void	PairSaver::add(pair<double, double> val, uint32_t time)
{
	buffer[bufEnd] = make_pair(val, time);
	incP(bufEnd);
	if(bufEnd == bufStart) incP(bufStart);
	//pairArr.push_back(1);
   // pairArr.push(make_pair(make_pair(speed.first, speed.second), time));
}

void PairSaver::incP(uint8_t &p)
{
	p = (p + 1) % SAVER_BUFFER_SIZE;
}


pair<double, double>	PairSaver::pop(uint32_t time)
{
   while(buffer[bufStart].second < time)
	 {
		 if(bufEnd - bufStart == 1 || (bufEnd == 0 && bufStart == SAVER_BUFFER_SIZE - 1))
			 break;
		 incP(bufStart);
	 }
    //    pairArr.pop();
    //return pairArr.front().first;
	return buffer[bufStart].first;
}
