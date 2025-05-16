#include "utils.hpp"

bool next_line_equal(std::ifstream &f, std::string s)
{
    std::string line;
    std::getline(f, line);
    return s.compare(line) == 0;
}