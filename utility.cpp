#include "utility.h"

int fd_translate_with_events(std::string event_prefix, std::string domain_filepath, std::string problem_filepath)
{
    char command[500];
    sprintf(command, "python2 ./translate/translate.py --add-events-with-prefix %s %s %s > /dev/null", event_prefix.c_str(), domain_filepath.c_str(), problem_filepath.c_str());
    int r = system(command);
    int translator_return = WEXITSTATUS(r);
    if (translator_return != 0)
    {
        std::cerr << "Fast Downward translation failed with exit code " << translator_return << "." << std::endl;
    }
    return translator_return;
}

int fd_translate(std::string domain_filepath, std::string problem_filepath)
{
    char command[500];
    sprintf(command, "python2 ./translate/translate.py %s %s > /dev/null", domain_filepath.c_str(), problem_filepath.c_str());
    int r = system(command);
    int translator_return = WEXITSTATUS(r);
    if (translator_return != 0)
    {
        std::cerr << "Fast Downward translation failed with exit code " << translator_return << "." << std::endl;
    }
    return translator_return;
}
