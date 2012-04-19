#include "ThreadedObject.h"

static int threadMain(void *arg)
{
    ThreadedObject *throbj = (ThreadedObject *)arg;
    while(throbj->isRunning()){
        if (!throbj->oneStep()) break;
    }
}

ThreadedObject::ThreadedObject() : 
    m_thread(NULL), m_isPausing(false), m_isRunning(false)
{
    m_sem = SDL_CreateSemaphore(0);
}

ThreadedObject::~ThreadedObject()
{
    SDL_DestroySemaphore(m_sem);
}

void ThreadedObject::pause(){
    m_isPausing = true;
}

void ThreadedObject::resume(){
    m_isPausing = false;
    SDL_SemPost(m_sem);
}

bool ThreadedObject::isPausing(){
    return m_isPausing;
}

bool ThreadedObject::isRunning(){
    return m_isRunning;
}

bool ThreadedObject::oneStep(){
    if (m_isPausing){
        SDL_SemWait(m_sem); 
    }
    return true;
}

void ThreadedObject::start()
{
    if (m_thread) return;
    m_isRunning = true;
    m_thread = SDL_CreateThread(threadMain, (void *)this);
}

void ThreadedObject::stop()
{
    m_isRunning = false;
    SDL_WaitThread(m_thread, NULL);
    m_thread = NULL;
}
