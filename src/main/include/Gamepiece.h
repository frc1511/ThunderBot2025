#pragma once

#include "Basic/Component.h"

class Gamepiece : public Component {
public:
    Gamepiece();
    ~Gamepiece();

    void process();
};