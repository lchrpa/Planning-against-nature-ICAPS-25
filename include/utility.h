#pragma once

#include <string>
#include <iostream>

int fd_translate_with_events(std::string event_prefix, std::string domain_filepath, std::string problem_filepath);
int fd_translate(std::string domain_filepath, std::string problem_filepath);
