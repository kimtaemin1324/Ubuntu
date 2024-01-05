#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
 
 int sum1 = 0;
 int sum2 = 0;
 int sum3 = 0;
// 쓰레드 동작시 실행될 함수
void *firstRun(void* arg)
{
    printf("\n");
    sleep(1);
    for(int i = 1; i <= 100; i++)
    {
		sum1 += i;
	}    
	printf("sum1 = %d\n", sum1);
	pthread_exit(NULL);
}

void *secondRun(void* arg)
{
    printf("\n");
    sleep(1);
    for(int i = 101; i <= 200; i++)
    {
		sum2 += i;
	}    
	printf("sum2 = %d\n", sum2);
	pthread_exit(NULL);
}

void *thirdRun(void* arg)
{
    printf("\n");
    sleep(1);
    for(int i = 201; i <= 300; i++)
    {
		sum3 += i;
	}    
	printf("sum3 = %d\n", sum3);
	pthread_exit(NULL);
}
 
int main()
{
    pthread_t first, second, third;
    int threadErr;
    
    
    // 쓰레드를 만들고 쓰레드 함수 실행
    if(threadErr = pthread_create(&first,NULL,firstRun,NULL))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d",threadErr);
    }
    
    if(threadErr = pthread_create(&second,NULL,secondRun,NULL))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d",threadErr);
    }
    
    if(threadErr = pthread_create(&third,NULL,thirdRun,NULL))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d",threadErr);
    }
    
    //while(1);
    pthread_join(first, NULL);
    pthread_join(second, NULL);
    pthread_join(third, NULL);
    
    printf("sum_total = %d\n", sum1 + sum2 + sum3);
}
