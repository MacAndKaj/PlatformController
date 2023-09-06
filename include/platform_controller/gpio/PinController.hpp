/**
  * Copyright (c) 2023 M. Kajdak. All rights reserved.
  */
#include <string>

class PinController
{
public:
    PinController(const std::string& device_path);
    virtual ~PinController();
    void set();
    void unset();
    void setupAsOutput(const std::string& label);

private:
    int m_fd{-1};
    const std::string m_device_path;
};
