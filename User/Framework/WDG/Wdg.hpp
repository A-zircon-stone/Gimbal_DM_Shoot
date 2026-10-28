//
// Created by 曹锐 on 2025/12/16.
//

#ifndef WDG_HPP
#define WDG_HPP
#include "Definitions.hpp"

class Wdg_t
{
public:
    int16_t cnt;
    OnlineState_t isOnline;

    Wdg_t() : cnt(0), isOnline(OnlineState_t::OFFLINE)
    {
    }

    void wdgf();
    void clear_wdg();
};


#endif