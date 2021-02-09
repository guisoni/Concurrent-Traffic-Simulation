#include <iostream>
#include <random>
#include <future>
#include <thread>
#include <algorithm>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */

 
template <typename T>
T MessageQueue<T>::receive()
{
    std::unique_lock<std::mutex> ulck(_mtx);
    _condition.wait(ulck,[this] { return !_queue.empty();}); 
     // remove last element from _queue
     T msg = std::move(_queue.back());
     _queue.pop_back();

     return msg; 
}

template <typename T>
void MessageQueue<T>::send(T &&msg)
{
        std::lock_guard<std::mutex> lck(_mtx);

        // add vector to queue
        _queue.clear();
        _queue.push_back(std::move(msg));
        _condition.notify_one(); // notify client after pushing msg to _queue

}


/* Implementation of class "TrafficLight" */

 
TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

void TrafficLight::waitForGreen()
{
    while(true){
        _currentPhase  =_msg.receive();
        if(_currentPhase == TrafficLightPhase::green){
            return;
        }        
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

void TrafficLight::simulate()
{
    // launch cycleThroughPhases function in a thread
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
void TrafficLight::cycleThroughPhases()
{    
    std::chrono::time_point<std::chrono::system_clock> lastTime;
    lastTime = std::chrono::system_clock::now();
        std::random_device rd;
        std::mt19937 eng(rd());
        std::uniform_int_distribution<> distr(4000, 6000);
        long cycleTime = distr(eng); // duration of a single simulation cycle in ms
    while (true)
    {    
        // sleep at every iteration to reduce CPU usage
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        // compute time difference to stop watch
        long deltaTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastTime).count();
        if (deltaTime >= cycleTime)
        {
            if(_currentPhase == TrafficLightPhase::green)
               _currentPhase = TrafficLightPhase::red;
            else
            _currentPhase = TrafficLightPhase::green;
            _msg.send(std::move(_currentPhase));
            lastTime = std::chrono::system_clock::now();
        }
    }
    
}