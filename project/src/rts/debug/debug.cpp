#include "debug.hpp"
#include "rts/file/file.hpp"

namespace NavigationSystemCode
{
    void Debug::clear()
    {
        File f;
        f.open("debug.log");
        f.write("---- Debug ----\n");
        f.close();
    }

    void Debug::log(const string &message)
    {
        write("Log: " + message);
    }

    void Debug::error(const string &message)
    {
        write("!!! Error: " + message);
    }

    void Debug::assert(const bool condition, const string &message)
    {
        if (condition)
        {
            write("!!! Assert: " + message);
        }
    }

    void Debug::index_assert(const int index, const int size, const string &message)
    {
        if (index < 0 || index >= size)
        {
            write("!!! Index out of range: " + to_string(index) + " | " + to_string(size) + " | " + message);
        }
    }

    void Debug::index_crash_assert(const int index, const int size, const string &message)
    {
        if (index < 0 || index >= size)
        {
            write("!!! Index out of range: " + to_string(index) + " | " + to_string(size) + " | " + message);
            exit(1);
        }
    }

    void Debug::write(const string &message)
    {
        File f;
        f.open_append("debug.log");
        f.write(message + "\n");
        f.close();
    }
}
