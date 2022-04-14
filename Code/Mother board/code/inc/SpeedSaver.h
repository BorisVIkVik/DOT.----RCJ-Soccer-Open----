#include <stdint.h>
#include <queue>
#include <utility>

using namespace std;

class SpeedSaver
{
    public:
        SpeedSaver();
        void add(double speed, uint32_t time);
        double pop(uint32_t time);
    private:
        queue<pair<double, uint32_t> > spArr;
};

void        SpeedSaver::add(double speed, uint32_t time)
{
    spArr.push(make_pair(speed, time));
}

double     SpeedSaver::pop(uint32_t time)
{
    while(spArr.front().second < time)
        spArr.pop();
    return spArr.front().first;
}