//
// Created by 曹锐 on 2025/12/16.
//

#include "Wdg.hpp"

void Wdg_t::clear_wdg()
{
    cnt = 0;
}

void Wdg_t::wdgf()
{
    if (cnt < 10)
    {
        cnt++;
        if (isOnline == OnlineState_t::OFFLINE) isOnline = OnlineState_t::ONLINE;
    }
    else isOnline = OnlineState_t::OFFLINE;
}