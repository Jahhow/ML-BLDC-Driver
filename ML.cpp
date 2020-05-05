#include <stdio.h>
#include <random>
#include <time.h>
#include <chrono>
using namespace std;
#define ARRLEN 64

default_random_engine engine = default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());
uniform_int_distribution<int> dis(0, 255);

uint8_t target[ARRLEN];
uint8_t pwm[ARRLEN];

int getRand()
{
    return dis(engine);
}

int Lose()
{
    int sum = 0;
    for (size_t i = 0; i < ARRLEN; i++)
    {
        int a = (int)pwm[i] - (int)target[i];
        if (a < 0)
            a = -a;
        sum += a;
    }
    return sum;
}

int main()
{
    for (size_t i = 0; i < ARRLEN; i++)
    {
        target[i] = getRand();
        printf("%3d ", target[i]);
    }

    while (1)
    {
        int epoch;
        printf("Epoch = ");
        scanf("%d", &epoch);
        if (epoch <= 0)
            break;
        
        for (size_t i = 1; i <= epoch; i++)
        {
            int v = 1;
            int lose = Lose();
            for (size_t iarr = 0; iarr < ARRLEN; iarr++)
            {
                pwm[iarr] += v;
                int newLose = Lose();
                int gradient = newLose - lose;
            }
        }
    }
    return 0;
}