#pragma once

#include <memory>

namespace asn
{
class mav;

class behaviour_planner
{
public:
    typedef std::shared_ptr<behaviour_planner> ptr;

public:
    behaviour_planner(mav &m);
    virtual ~behaviour_planner();

private:
    mav & mav_;
};

}
