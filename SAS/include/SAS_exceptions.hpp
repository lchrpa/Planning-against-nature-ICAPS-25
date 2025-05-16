#ifndef ___SAS_EXCEPTIONS___
#define ___SAS_EXCEPTIONS___

#include <exception>

class SAS_not_found : public std::exception
{

private:
    std::string what_message;

public:
    SAS_not_found(const std::string message) : what_message(message){};

    virtual const char *what() noexcept
    {
        return what_message.c_str();
    }
};

class SAS_file_error : public std::exception
{

private:
    std::string what_message;

public:
    SAS_file_error(const std::string message) : what_message(message){};

    virtual const char *what() noexcept
    {
        return what_message.c_str();
    }
};

class SAS_out_of_range : public std::exception
{

private:
    std::string what_message;

public:
    SAS_out_of_range(const std::string message) : what_message(message){};

    virtual const char *what() noexcept
    {
        return what_message.c_str();
    }
};

#endif