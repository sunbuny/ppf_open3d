#include <cassert>
#include <CPUTimer.h>

#include <stdio.h>
#include <iomanip>

#include <iostream>
#include <sstream>

using namespace std;

CPUTimer::CPUTimer(bool print) : startWasSet(false), stopWasSet(true),print(print)  {
}

CPUTimer::~CPUTimer() {
}

void CPUTimer::getTime(struct timespec *ts){
#ifdef __MACH__
    //https://gist.github.com/jbenet/1087739
  // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;
#else
#ifdef _WIN32
    clock_gettime(ts);
#else
    clock_gettime(CLOCK_REALTIME, ts);
#endif
#endif

}

void CPUTimer::tic() {
    assert(stopWasSet && !startWasSet);

    getTime(&startTime);
    startWasSet = true;
    stopWasSet = false;
}

timespec CPUTimer::toc() {
    assert(startWasSet && !stopWasSet);
    getTime(&endTime);

    timespec temp;

    if ((endTime.tv_nsec - startTime.tv_nsec)<0) {
        temp.tv_sec = endTime.tv_sec - startTime.tv_sec-1;
        temp.tv_nsec = 1000000000 + endTime.tv_nsec - startTime.tv_nsec;
    } else {
        temp.tv_sec = endTime.tv_sec - startTime.tv_sec;
        temp.tv_nsec = endTime.tv_nsec - startTime.tv_nsec;
    }

    stopWasSet = true;
    startWasSet = false;

    return temp;
}

float CPUTimer::tocSeconds(){
    timespec elapsedTimeCPUTimer = this->toc();
    return elapsedTimeCPUTimer.tv_sec + elapsedTimeCPUTimer.tv_nsec / 1e9f;
}

void CPUTimer::toc(std::string name){
    timespec elapsedTimeCPUTimer = this->toc();
    if(print)
    {
        cout << endl;
        cout << "=====  TIMING[" << name << "] is ";
        cout << elapsedTimeCPUTimer.tv_sec + elapsedTimeCPUTimer.tv_nsec / 1e9f << " s" << endl;
        cout << endl;
    }
    float seconds=elapsedTimeCPUTimer.tv_sec + elapsedTimeCPUTimer.tv_nsec / 1e9f;
    std::pair<std::string,float> time_stamp;
    time_stamp.first = name;
    time_stamp.second = seconds;
    timingsMap.push_back(time_stamp);
}

void CPUTimer::printAllTimings(){
    typedef std::map<std::string,float>::iterator it_type;
    cout<<"=====  TIMINGS ===="<<endl;
    std::sort(timingsMap.begin(),timingsMap.end(),[](std::pair<std::string,float> &a, std::pair<std::string,float> &b) {
        return a.second > b.second;
    });
    for(auto it:timingsMap) {
        cout<< left << setw(50) << it.first<<":\t";
        printf("%0.3f\n",it.second);
    }
}

string CPUTimer::getHeader(){
//    typedef std::pair<std::string,float>::iterator it_type;
    std::stringstream ss;

    for(auto it:timingsMap) {
        ss<<","<<it.first;
    }
    return ss.str();
}

string CPUTimer::getMeasurements(){
    std::stringstream ss;

    for(auto it:timingsMap) {
        ss<<","<<it.second;
    }
    return ss.str();
}

