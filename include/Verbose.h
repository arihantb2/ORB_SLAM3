#pragma once

#include <iostream>

#include <sstream>

#include <string>

namespace ORB_SLAM3
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET = 0,
        VERBOSITY_NORMAL = 1,
        VERBOSITY_VERBOSE = 2,
        VERBOSITY_VERY_VERBOSE = 3,
        VERBOSITY_DEBUG = 4
    };

    static eLevel th;

public:
    class VerboseStream
    {
    public:
        explicit VerboseStream(eLevel lev) : level(lev) {}

        ~VerboseStream() { Flush(); }

        template <typename T>
        VerboseStream& operator<<(const T& value)
        {
            if (level <= th)
                buffer << value;
            return *this;
        }

        VerboseStream& operator<<(std::ostream& (*manip)(std::ostream&))
        {
            if (level > th)
                return *this;

            manip(buffer);
            if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl) ||
                manip == static_cast<std::ostream& (*)(std::ostream&)>(std::flush))
            {
                Flush();
            }
            return *this;
        }

    private:
        void Flush()
        {
            if (level > th)
                return;

            const std::string out = buffer.str();
            if (!out.empty())
            {
                std::cout << out;
                std::cout.flush();
                buffer.str("");
                buffer.clear();
            }
        }

        eLevel level;
        std::ostringstream buffer;
    };

    static VerboseStream Print(eLevel lev = VERBOSITY_NORMAL) { return VerboseStream(lev); }

    static void PrintMess(std::string str, eLevel lev)
    {
        if (lev <= th)
            std::cout << str << std::endl;
    }

    static void SetTh(eLevel _th) { th = _th; }
};

}