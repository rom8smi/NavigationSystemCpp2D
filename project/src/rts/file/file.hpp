#ifndef FILE_HPP
#define FILE_HPP

#include <iostream>
#include <fstream>

using namespace std;

namespace NavigationSystemCode
{
    struct File
    {
        ofstream file;

        void open(const string &file_name);
        void open_append(const string &file_name);
        void write(const string &data);
        void close();
    };
}

#endif
