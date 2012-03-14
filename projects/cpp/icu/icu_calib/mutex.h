#ifndef MUTEX_H
#define MUTEX_H

#include <pthread.h>
#include <cmath>

class Mutex {
public:
        Mutex();
        void lock();
        void unlock();
private:
        pthread_mutex_t m_mutex;
};

#endif // MUTEX_H
