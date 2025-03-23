#pragma once

class Action {
public:
    virtual ~Action() = default;

    /**
     * The result of the action.
     */
    enum Result {
        DONE = 0, // Signifies that the action is done.
        WORKING = 1, // Signifies that the trajectory should wait for the action.
        DONE_BUT_LINEUP_STILL_NEEDS_TO_HAPPEN = 2
    };

    virtual Result process() = 0;
};