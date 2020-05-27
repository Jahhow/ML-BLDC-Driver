#include <stdio.h>
#include <math.h>
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

int Loss()
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

void printArray(uint8_t a[], size_t size)
{
    for (size_t i = 0; i < size; i++)
        printf("%3u ", (uint32_t)a[i]);
    putchar('\n');
}

void printArrayDiff(uint8_t a[], uint8_t b[], size_t size)
{
    printf("  ArrayDifference: ");
    for (size_t i = 0; i < size; i++)
        printf("%3d ", (int)a[i] - (int)b[i]);
    putchar('\n');
}

int main()
{
    for (size_t i = 0; i < ARRLEN; i++)
    {
        target[i] = getRand();
        pwm[i] = 127;
    }
    printArrayDiff(target, pwm, ARRLEN);

    while (1)
    {
        int epoch, innerEpoch = 5;
        int lose;
        printf("\nEpoch = ");
        if (scanf("%d", &epoch) != 1)
            break;
        if (epoch <= 0)
            break;

        for (size_t i = 1; i <= epoch; i++)
        {
            printf("  Epoch %2d: ", i);
            for (size_t iarr = 0; iarr < ARRLEN; iarr++)
            {
                int direction = 1;
                int loseDiff;
                int v = 1;
                lose = Loss();
                //printf("\n iarr: %u, Loss %d ", iarr, lose);
                for (size_t iInnerEpoch = 0; iInnerEpoch < innerEpoch; iInnerEpoch++)
                {
                    uint8_t oldCurPwm = pwm[iarr];
                    pwm[iarr] = max(min((int)oldCurPwm + v, 255), 0);
                    int newLose = Loss();
                    loseDiff = newLose - lose;
                    //printf("%d ", newLose);
                    if (loseDiff >= 0)
                    {
                        pwm[iarr] = oldCurPwm;
                        direction = -direction;
                    }
                    lose = newLose;
                    if (loseDiff == 0)
                        break;
                    v = direction * abs(loseDiff);
                }
            }
            printf("Loss: %4d\n", Loss());
        }
        puts("  Done");
        printArrayDiff(target, pwm, ARRLEN);
    }
    return 0;
}