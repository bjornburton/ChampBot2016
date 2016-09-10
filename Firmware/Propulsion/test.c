#include<stdio.h>
#include<stdlib.h>
#include<stdint.h>
int main()
{

int16_t offset[]={-11, 4, 5, -3, 12};
printf("%d\n",sizeof(offset)/sizeof(offset[0]));


}


