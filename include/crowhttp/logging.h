#pragma once

#include "crowhttp/settings.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <sstream>
#include <string>

namespace crowhttp
{
    enum class LogLevel
    {
#ifndef ERROR
#ifndef DEBUG
        DEBUG = 0,
        INFO,
        WARNING,
        ERROR,
        CRITICAL,
#endif
#endif

        Debug = 0,
        Info,
        Warning,
        Error,
        Critical,
    };

    class ILogHandler
    {
    public:
        virtual ~ILogHandler() = default;

        virtual void log(const std::string& message, LogLevel level) = 0;
    };

    class CerrLogHandler : public ILogHandler
    {
    public:
        void log(const std::string &message, LogLevel level) override
        {
            std::string log_msg;
            log_msg.reserve(message.length() + 1+32+3+8+2);
            log_msg
                .append("(")
                .append(timestamp())
                .append(") [");

            switch (level)
            {
                case LogLevel::Debug:
                    log_msg.append("DEBUG   ");
                    break;
                case LogLevel::Info:
                    log_msg.append("INFO    ");
                    break;
                case LogLevel::Warning:
                    log_msg.append("WARNING ");
                    break;
                case LogLevel::Error:
                    log_msg.append("ERROR   ");
                    break;
                case LogLevel::Critical:
                    log_msg.append("CRITICAL");
                    break;
            }

            log_msg.append("] ")
            .append(message);

            std::cerr << log_msg << std::endl;
        }

    private:
        static std::string timestamp()
        {
            char date[32];
            time_t t = time(0);

            tm my_tm;

#if defined(_MSC_VER) || defined(__MINGW32__)
#ifdef CROW_USE_LOCALTIMEZONE
            localtime_s(&my_tm, &t);
#else
            gmtime_s(&my_tm, &t);
#endif
#else
#ifdef CROW_USE_LOCALTIMEZONE
            localtime_r(&t, &my_tm);
#else
            gmtime_r(&t, &my_tm);
#endif
#endif

            size_t sz = strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &my_tm);
            return std::string(date, date + sz);
        }
    };

    class logger
    {
    public:
        logger(LogLevel level):
          level_(level)
        {}
        ~logger()
        {
#ifdef CROW_ENABLE_LOGGING
            if (level_ >= get_current_log_level())
            {
                get_handler_ref()->log(stringstream_.str(), level_);
            }
#endif
        }

        //
        template<typename T>
        logger& operator<<(T const& value)
        {
#ifdef CROW_ENABLE_LOGGING
            if (level_ >= get_current_log_level())
            {
                stringstream_ << value;
            }
#endif
            return *this;
        }

        //
        static void setLogLevel(LogLevel level) { get_log_level_ref() = level; }

        static void setHandler(ILogHandler* handler) { get_handler_ref() = handler; }

        static LogLevel get_current_log_level() { return get_log_level_ref(); }

    private:
        //
        static LogLevel& get_log_level_ref()
        {
            static LogLevel current_level = static_cast<LogLevel>(CROW_LOG_LEVEL);
            return current_level;
        }
        static ILogHandler*& get_handler_ref()
        {
            static CerrLogHandler default_handler;
            static ILogHandler* current_handler = &default_handler;
            return current_handler;
        }

        //
        std::ostringstream stringstream_;
        LogLevel level_;
    };
} // namespace crowhttp

#define CROW_LOG_CRITICAL                                                  \
    if (crowhttp::logger::get_current_log_level() <= crowhttp::LogLevel::Critical) \
    crowhttp::logger(crowhttp::LogLevel::Critical)
#define CROW_LOG_ERROR                                                  \
    if (crowhttp::logger::get_current_log_level() <= crowhttp::LogLevel::Error) \
    crowhttp::logger(crowhttp::LogLevel::Error)
#define CROW_LOG_WARNING                                                  \
    if (crowhttp::logger::get_current_log_level() <= crowhttp::LogLevel::Warning) \
    crowhttp::logger(crowhttp::LogLevel::Warning)
#define CROW_LOG_INFO                                                  \
    if (crowhttp::logger::get_current_log_level() <= crowhttp::LogLevel::Info) \
    crowhttp::logger(crowhttp::LogLevel::Info)
#define CROW_LOG_DEBUG                                                  \
    if (crowhttp::logger::get_current_log_level() <= crowhttp::LogLevel::Debug) \
    crowhttp::logger(crowhttp::LogLevel::Debug)
