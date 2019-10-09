#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include <vector>
#define N 2
    
using namespace std;
 



//  g++ test.cpp  -lpthread -g


vector<int> g_v;
pthread_mutex_t mutex;
 
void* fun(void *p)
{
    for(int i = 0; i < 100000; i++)
    {
    	pthread_mutex_lock(&mutex);
        g_v.push_back(i);
		pthread_mutex_unlock(&mutex);
    }
 
    return NULL;
}
 
int main()
{
    pthread_t threads[N];
    pthread_mutex_init(&mutex, NULL);
 
    for(int i = 0; i <  N; i++)
    {
        pthread_create(&threads[i], NULL, fun, NULL);
    }
	
    for(int i = 0; i <  N; i++)
    {
        pthread_join(threads[i],NULL);
    }
 
 	cout << "ok" << endl;
	
	return 0;
}