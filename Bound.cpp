//
//  Bound.cpp
//  simulation
//
//  Created by skyer on 14-4-15.
//  Copyright (c) 2014年 skyer. All rights reserved.
//

#include "Bound.h"

Bound::Bound()
    : kr(0)
{}

Bound::~Bound()
{}

void Bound::setKr(double _kr)
{
    kr = _kr;
}

void Bound::setKf(double _kf)
{
    kf = _kf;
}
