#include "file.hpp"

namespace NavigationSystemCode
{
    void File::open(const string &file_name)
    {
        file.open(file_name);
    }

    void File::open_append(const string &file_name)
    {
        file.open(file_name, std::ios_base::app);
    }

    void File::write(const string &data)
    {
        file.write(data.c_str(), data.length());
    }

    void File::close()
    {
        file.close();
    }
}