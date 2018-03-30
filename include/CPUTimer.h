//
// Created by sun on 18-3-30.
//

#pragma once

#include <iostream>
#include <string>
#include <map>
#include <time.h>
#include <algorithm>
#include <functional>
#include <array>

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME CLOCK_MONOTONIC
#endif

#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif

#ifdef _WIN32
#include <wintime.h>
#else
#include <sys/time.h>
#endif

class CPUTimer {
private:
    timespec startTime;
    timespec endTime;

    bool startWasSet;
    bool stopWasSet;
    bool print;

    void getTime(struct timespec *ts);

    std::vector<std::pair<std::string,float>> timingsMap;

public:
    CPUTimer(bool print);
    ~CPUTimer();

    void tic();
    timespec toc();
    void toc(std::string name);

    float tocSeconds();

    void printAllTimings();

    std::string getHeader();
    std::string getMeasurements();
};