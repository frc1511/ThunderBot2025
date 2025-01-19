#pragma once

#include "Basic/Component.h"

class Gamepiece : public Component {
public:
    Gamepiece();
    virtual ~Gamepiece();

    virtual void process();
};