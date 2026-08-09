#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <string>

using namespace std;

namespace NavigationSystemCode
{
    struct Debug
    {
        static void clear();
        static void log(const string &message);
        static void error(const string &message);
        static void assert(const bool condition, const string &message);
        static void index_assert(const int index, const int size, const string &message);
        static void index_crash_assert(const int index, const int size, const string &message);
        static void write(const string &message);
    };
}

#endif
