// gcc main.c -o main

#include <stdio.h>
#include <stdlib.h>
#include "car_lib.h"

void main(void)
{
    int channel;
    int data;
    char sensor;

    CarControlInit();

    // 5. distance sensor --------------------------------------------------------
    while(1){
        printf("\n\ndistance sensor\n");
        data = DistanceSensor(1);
        printf("channel = %d, distance = 0x%04X(%d) \n", channel, data, data);
        scanf("%c", &sensor);
    }
    
}

