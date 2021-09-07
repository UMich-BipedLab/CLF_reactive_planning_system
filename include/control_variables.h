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
#ifndef CONTROL_VARIABLES
#define CONTROL_VARIABLES 


#include <iostream>
#include <string>



namespace bipedlab
{
    typedef struct control_variables
    {
        double vx;
        double vy;
        double vr;
        double vd;
        double omega;
        double heading;

        control_variables(double vx, double vy, double vr, double vd, 
                          double omega, double heading) :
            vx(vx), vy(vy), heading(heading), omega(omega), vr(vr), vd(vd) { }

        control_variables(void):
            vx(0), vy(0), heading(0), omega(0), vr(0), vd(0) { }

        control_variables(double heading):
            vx(0), vy(0), heading(heading), omega(0), vr(0), vd(0) { }

        control_variables(double heading, double omega):
            vx(0), vy(0), heading(heading), omega(omega), vr(0), vd(0) { }

        void print(void) const
        {
            std::cout << "vx: " << this->vx << ", vy: " << this->vy 
                      << ", omega: " << this->omega
                      << ", heading: " << this->heading << std::endl;
        }

        std::string returnMemberString(void) const

        {
            return "vx: " + std::to_string(this->vx) + 
                   ", vy: " + std::to_string(this->vy) +
                   ", omega: " + std::to_string(this->omega) +
                   ", heading: " + std::to_string(this->heading);
        }
    } control_variables_t;
    
} /* bipedlab */ 

#endif /* ifndef CONTROL_VARIABLES */
