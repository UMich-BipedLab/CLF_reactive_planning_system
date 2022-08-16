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
// #pragma once
#ifndef DEBUGGER_H
#define DEBUGGER_H


#include <iostream>
#include <string>
#include <iomanip>      // std::setprecision


extern int DEBUG_LEVEL;
enum text_color_t
{
    BLACK = 30, K = 30,
    RED = 31, R = 31,
    GREEN = 32, G = 32,
    YELLOW = 33, Y = 33, 
    BLUE = 34, B = 34,
    MEGENTA = 35, M = 35,
    CYAN = 36, C = 36,
    WHITE = 37, W = 37,

    // bright color
    BRIGHTBLACK = 90, BK = 90,
    BRIGHTRED = 91, BR = 91,
    BRIGHTGREEN = 92, BG = 92,
    BRIGHTYELLOW = 93, BY = 93, 
    BRIGHTBLUE = 94, BB = 94,
    BRIGHTMEGENTA = 95, BM = 95,
    BRIGHTCYAN = 96, BC = 96,
    BRIGHTWHITE = 97, BW = 97,
};


enum text_front_t
{
    NORMAL = 0, N = 0,
    BOLD = 1,
    FAINT = 2, F = 2,
    ITALIC = 3, I = 3,
    SBLINK = 5,
    RBLINK = 6, FBLINK = 6,

};

namespace bipedlab
{
namespace debugger
{
    // template <class T>
    // void debugOutputStream(std::ostream& out, int level = 0)
    // {
    //     if (level >= DEBUG_LEVEL)
    //         std::cout << out << std::endl;
    // }
    
    // the higher level, the less outputs will be
    // i.e., the lower level, the less important the message is
    // assign higher level if the message is important, 
    template <class T>
    void debugOutput(const std::string text, const T& value, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << text << value << std::endl;
    }

    inline 
    void debugTextOutput(std::string text, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << text << std::endl;
    }


    template <class T> 
    void debugColorOutput(const std::string text, 
            const T& value, int level = 0, 
            text_color_t color = RED, text_front_t font = N)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << "\033[" + std::to_string(font) + ";" + 
                         std::to_string(color) + "m" 
                      << text 
                      << value 
                      << "\033[0m"
                      << std::endl;
    }


    template <class T>
    void debugWarningOutput(const std::string text, const T& value, int level = 0, 
            text_color_t color = YELLOW)
    {
        if (level >= DEBUG_LEVEL)
        {
            debugColorOutput("[Warning]--" + text, value, level, Y, N);
        }
    }


    template <class T> 
    void debugColorOutputPrecision(const std::string text, 
            const T& value, int level = 0, const size_t precision = 3,
            text_color_t color = RED, text_front_t font = N)
    {
        if (level >= DEBUG_LEVEL)
            std::cout << "\033[" + std::to_string(font) + ";" + 
                         std::to_string(color) + "m" 
                      << text 
                      << std::setprecision(precision) << std::fixed
                      << value 
                      << "\033[0m"
                      << std::endl;
    }

    inline
    void debugColorTextOutput(const std::string text, 
            int level = 0, text_color_t color = RED, text_front_t font = N)
    {
        if (level >= DEBUG_LEVEL)
            debugColorOutput(text, "", level, color, font);
    }
    

    inline 
    void debugTitleTextOutput(const std::string script, 
            const std::string& title, int level = 0, 
            text_color_t color = RED, text_front_t font = BOLD, 
            char seperator = '=')
    {
        size_t num_seperator = (size_t) std::max((70.0 - title.size()) / 2.0, 2.0);
        if (level >= DEBUG_LEVEL)
            std::cout << "\033[" + std::to_string(font) + ";" + 
                         std::to_string(color) + "m" 
                      << script 
                      << " "
                      << std::string(num_seperator, seperator) 
                      << " "
                      << title 
                      << " "
                      << std::string(num_seperator, seperator) 
                      << "\033[0m"
                      << std::endl;
    }


    // std::string error_msg = "No such mode: " +
    //     std::to_string();
    // debugger::debugExitColor(error_msg, __LINE__, __FILE__);
    inline 
    void debugExitColor(std::string error_msg, 
            int line_number, std::string file_name,
            int level = 20, text_color_t color = BR, text_front_t font = BOLD)
    {
        if (level >= DEBUG_LEVEL)
        {
            debugger::debugColorOutput(error_msg, "", 20, color, font);
            std::string error_location = "This error occured at " + file_name + 
                ":" + std::to_string(line_number);
            debugger::debugColorOutput(error_location, "", 20, color, font);
            exit(-1);
        }
        
    }



    inline 
    void debugTextCerr(std::string text, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
        {
            std::cerr << text << std::endl;
            exit(-1);
        }
        
    }
    
    template <class T>
    void debugCerr(std::string text, const T& value, int level = 0)
    {
        if (level >= DEBUG_LEVEL)
        {
            std::cerr << text << value << std::endl;
            exit(-1);
        }
        
    }

} /* debugger */ 
} /* bipedlab */ 
#endif /* ifndef DEBUGGER_H */
