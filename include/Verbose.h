#pragma once

#include <atomic>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
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
        VERBOSITY_DEBUG = 2
    };

    static std::atomic<eLevel> th;
    static std::mutex cout_mutex;
    static std::unique_ptr<std::ofstream> log_file_;
    static std::atomic<bool> console_enabled;

    static const char* LevelToString(eLevel lev)
    {
        switch (lev)
        {
            case VERBOSITY_QUIET: return "QUIET";
            case VERBOSITY_NORMAL: return "NORMAL";
            case VERBOSITY_DEBUG: return "DEBUG";
        }
        return "UNKNOWN";
    }

    static void SetLogFile(const std::string& path)
    {
        std::lock_guard<std::mutex> lock(cout_mutex);
        log_file_.reset();
        if (!path.empty())
        {
            log_file_ = std::make_unique<std::ofstream>(path, std::ios::out | std::ios::trunc);
        }
    }

    static void SetConsole(bool enabled) { console_enabled.store(enabled, std::memory_order_relaxed); }

public:
    class VerboseStream
    {
    public:
        explicit VerboseStream(eLevel lev) : level(lev) {}

        ~VerboseStream() { Flush(); }

        template <typename T>
        VerboseStream& operator<<(const T& value)
        {
            buffer << value;
            return *this;
        }

        VerboseStream& operator<<(std::ostream& (*manip)(std::ostream&))
        {
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
            const std::string out = buffer.str();
            if (out.empty())
            {
                return;
            }
            std::lock_guard<std::mutex> lock(cout_mutex);
            if (log_file_ && *log_file_)
            {
                *log_file_ << "[" << LevelToString(level) << "] " << out;
                if (out.back() != '\n')
                {
                    *log_file_ << '\n';
                }
                log_file_->flush();
            }
            if (console_enabled.load(std::memory_order_relaxed) && level <= th.load(std::memory_order_relaxed))
            {
                std::cout << out;
                if (out.back() != '\n')
                {
                    std::cout << '\n';
                }
                std::cout.flush();
            }
            buffer.str("");
            buffer.clear();
        }

        eLevel level;
        std::ostringstream buffer;
    };

    static VerboseStream Print(eLevel lev = VERBOSITY_NORMAL) { return VerboseStream(lev); }

    static void PrintMess(const std::string& str, eLevel lev)
    {
        std::lock_guard<std::mutex> lock(cout_mutex);
        if (log_file_ && *log_file_)
        {
            *log_file_ << "[" << LevelToString(lev) << "] " << str << std::endl;
            log_file_->flush();
        }
        if (console_enabled.load(std::memory_order_relaxed) && lev <= th.load(std::memory_order_relaxed))
        {
            std::cout << str << std::endl;
        }
    }

    static void SetTh(eLevel _th) { th.store(_th, std::memory_order_relaxed); }
};

}  // namespace ORB_SLAM3