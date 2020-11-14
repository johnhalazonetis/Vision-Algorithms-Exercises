#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace boost::filesystem;
using namespace boost::lambda;

int main(int argc, char** argv)
{
    path datapath( "../data/exercise1/images/" );

    int cnt = count_if(directory_iterator(datapath), directory_iterator(), static_cast<bool(*)(const path&)>(is_regular_file) );
    cout << cnt << endl;

    return 0;
}