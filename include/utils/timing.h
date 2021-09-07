/* Copyright (C) 2013-2020, The Regents of The University of Michigan.
 * All rights reserved.
 * This software was developed in the Biped Lab (https://www.biped.solutions/)
 * under the direction of Jessy Grizzle, grizzle@umich.edu. This software may
 * be available under alternative licensing terms; contact the address above.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the Regents of The University of Michigan.
 *
 * AUTHOR: Bruce JK Huang (bjhuang@umich.edu)
 * WEBSITE: https://www.BrucebotStudio.com/
 */
#pragma once
#ifndef TIMMING
#define TIMMING 


#include <math.h>
#include <iostream>
#include <chrono>
#include <ctime>


namespace bipedlab
{
namespace timing
{

    inline
    std::clock_t
    getCurrentCPUTime()
    {
        return std::clock();
    }

    inline
    double spendCPUTime(const std::clock_t &t_start, 
                        const std::clock_t t_end = std::clock())
    {
        return (((double) (t_end - t_start))/CLOCKS_PER_SEC);
    }

    inline
    double spendCPUHz(const std::clock_t &t_start, 
                      const std::clock_t t_end = std::clock()){
        return 1.0/spendCPUTime(t_start, t_end);
    }

    inline
    double printSpendCPUHz(
            const std::clock_t &t_start, std::string txt, 
            const std::clock_t t_end = std::clock()){
        std::cout << std::fixed << std::setprecision(2)
                  << txt << spendCPUHz(t_start, t_end) << " [Hz]"  << std::endl;
    }

    inline
    double printSpendCPUHz(
            const std::clock_t &t_start,
            const std::clock_t &t_end = std::clock()){
        std::string text = "CPU time used: ";
        printSpendCPUHz(t_start, text, t_end);
    }



    // std::chrono::steady_clock::time_point clock_start = std::chrono::steady_clock::now();
    // std::chrono::duration<double> duration =
    //     std::chrono::steady_clock::now() - clock_start;
    inline
    std::chrono::steady_clock::time_point
    getCurrentTime()
    {
        return std::chrono::steady_clock::now();
    }
    
    inline
    double spendElapsedTime(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        std::chrono::duration<double> duration = t_end - t_start;
        return duration.count();
    }

    inline
    double spendElapsedTimeMilli(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        return 1e3*spendElapsedTime(t_end, t_start);
    }

    inline
    double spendElapsedHz(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        return 1.0/spendElapsedTime(t_start, t_end);
    }

    inline
    double printSpendElapsedHz(
            const std::chrono::steady_clock::time_point &t_start,
            std::string txt,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        std::cout << std::fixed << std::setprecision(2)
                  << txt << spendElapsedHz(t_start, t_end) << " [Hz]"  << std::endl;
    }

    inline
    double printSpendElapsedHz(
            const std::chrono::steady_clock::time_point &t_start,
            const std::chrono::steady_clock::time_point &t_end = getCurrentTime())
    {
        std::string text = "Elapsed time: ";
        printSpendElapsedHz(t_start, text, t_end);
    }

        
} /* timing */ 
    
} /* bipedlab */ 

#endif /* ifndef TIMMING */
