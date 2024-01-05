#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>

pthread_mutex_t mutex; 
int sum1 = 0;

void *firstRun(void* arg)
{
    for(int i = 1; i <= 100; i++)
    {
		pthread_mutex_lock(&mutex);
		sum1 += i;
		pthread_mutex_unlock(&mutex);
	}    
	printf("sum1 = %d\n", sum1);
	pthread_exit(NULL);
}

void *secondRun(void* arg)
{
    for(int i = 101; i <= 200; i++)
    {
		pthread_mutex_lock(&mutex);
		sum1 += i;
		pthread_mutex_unlock(&mutex);
	}    
	printf("sum1 = %d\n", sum1);
	pthread_exit(NULL);
}

void *thirdRun(void* arg)
{
    for(int i = 201; i <= 300; i++)
    {
		pthread_mutex_lock(&mutex);
		sum1 += i;
		pthread_mutex_unlock(&mutex);
	}    
	printf("sum1 = %d\n", sum1);
	pthread_exit(NULL);
}
 
int main()
{
    pthread_t first, second, third;
    int threadErr;
    
    if(threadErr = pthread_create(&first,NULL,firstRun,NULL))
    {
        printf("Thread Err = %d",threadErr);
    }
    
    if(threadErr = pthread_create(&second,NULL,secondRun,NULL))
    {
        printf("Thread Err = %d",threadErr);
    }
    
    if(threadErr = pthread_create(&third,NULL,thirdRun,NULL))
    {
        printf("Thread Err = %d",threadErr);
    }
    
    //while(1);
    pthread_join(first, NULL);
    pthread_join(second, NULL);
    pthread_join(third, NULL);
    
    printf("sum_total = %d\n", sum1);
    pthread_mutex_destroy(&mutex);
    exit(0);
}
