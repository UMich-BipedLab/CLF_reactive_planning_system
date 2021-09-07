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
#ifndef COST_H
#define COST_H


#include <iosfwd>
#include <iostream>

namespace bipedlab
{
    
class Cost
{
private:
    // no total cost here on purpose
    double distance_;
    double height_;

public:
    Cost(void);
    explicit Cost(const double& distance_cost, const double& height_cost);
    explicit Cost(const double& distance_cost);

    Cost(const Cost& t_cost);

    virtual ~Cost();


    double getTotalCost(void) {return distance_ + height_; };
    double getDistanceCost(void) { return distance_; };
    double getHeightCost(void) { return height_; };


    // operators
    inline Cost operator+ (const Cost& t_cost);
    inline bool operator< (const Cost& rhs) const;
};

} /* bipedlab */ 
#endif /* COST_H */






// #ifndef COST_H
// #define COST_H
// 
// 
// #include <iosfwd>
// #include <iostream>
// 
// 
// typedef struct cost
// {
//     // no total cost here on purpose
//     double distance;
//     double height;
// 
// 
//     explicit cost(const double& distance_cost, const double& height_cost) :
//         distance(distance_cost), 
//         height(height_cost) { }
// 
//     explicit cost(const double& distance_cost) :
//          distance(distance_cost), height(0) { }
// 
//     cost(void) : distance(0), height(0) { };
// 
//     cost(const cost& t_cost) :
//         distance(t_cost.distance), height(t_cost.height) { }
// 
//     inline cost operator+ (const cost& t_cost) 
//     {
//         return cost(t_cost.distance + distance, t_cost.height + height);
//     }
// 
//     inline bool operator< (const cost& rhs) const
//     {
//         return distance + height < rhs.distance + rhs.height;
//     }
// 
//     double getTotalCost(void) {return distance + height; };
// 
// } cost_t;
// 
// std::ostream& operator<<(std::ostream& out, const cost& t_cost);
// 
// 
// #endif /* COST_H */


