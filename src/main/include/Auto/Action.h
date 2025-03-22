#pragma once

class Action {
public:
    virtual ~Action() = default;

    virtual std::string getToken() = 0;

    /**
     * The result of the action.
     */
    enum Result {
        DONE = 0, // Signifies that the action is done.
        WORKING = 1, // Signifies that the trajectory should wait for the action.
    };

    virtual Result process() = 0;
};