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
#ifndef CSV
#define CSV 

#include <iostream>
#include <fstream>

class CSVFile;

inline static CSVFile& endrow(CSVFile& file);
inline static CSVFile& flush(CSVFile& file);

class CSVFile
{
    std::ofstream fs_;
    const std::string separator_;
public:
    CSVFile(const std::string filename, const std::string separator = ";")
        : fs_()
        , separator_(separator)
    {
        fs_.exceptions(std::ios::failbit | std::ios::badbit);
        fs_.open(filename);
    }

    CSVFile(const std::string separator = ";")
        : fs_()
        , separator_(separator)
    {
        fs_.exceptions(std::ios::failbit | std::ios::badbit);
    }

    ~CSVFile()
    {
        flush();
        fs_.close();
    }

    void open(const std::string filename)
    {
        fs_.open(filename);
    }

    void flush()
    {
        fs_.flush();
    }

    void endrow()
    {
        fs_ << std::endl;
    }

    CSVFile& operator << ( CSVFile& (* val)(CSVFile&))
    {
        return val(*this);
    }

    CSVFile& operator << (const char * val)
    {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    CSVFile& operator << (const std::string & val)
    {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    template<typename T>
    CSVFile& operator << (const T& val)
    {
        fs_ << val << separator_;
        return *this;
    }
};


inline static CSVFile& endrow(CSVFile& file)
{
    file.endrow();
    return file;
}

inline static CSVFile& flush(CSVFile& file)
{
    file.flush();
    return file;
}

#endif /* ifndef CSV */
