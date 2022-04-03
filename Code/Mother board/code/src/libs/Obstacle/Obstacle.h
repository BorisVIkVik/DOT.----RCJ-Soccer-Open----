struct Obstacle
{
    int _x;
    int _y;
    int _n;
    int** _vecField;
    Obstacle(int** vecField, int x, int y, int n)
    {
        _x = x;
        _y = y;
        _n = n;
        _vecField = vecField;
    }
};
