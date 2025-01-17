#pragma once

#include <filesystem>
#include <string>
#include <functional>

class Parser {
public:
    using Iter = std::string::const_iterator;
    using ConditionFunc = std::function<bool(char)>;

    /**
     * Returns the contents of a file as a string.
     */
    static std::string getFile(std::filesystem::path path);

    /**
     * Counts the number of characters while a condition is met.
     */
    static std::ptrdiff_t countWhile(Iter currIter, Iter endIter, ConditionFunc condFunc);

    /**
     * Parses a string while a condition is met.
     */
    static std::string parseWhile(Iter& currIter, Iter endIter, ConditionFunc condFunc);

    /**
     * Counts the number of characters until a character is met.
     */
    static std::ptrdiff_t countUntil(Iter currIter, Iter endIter, std::string chars);

    /**
     * Parses a string until a character is met.
     */
    static std::string parseUntil(Iter& currIter, Iter endIter, std::string chars);

    /**
     * Parses a number from a string.
     */
    static double parseNumber(Iter& currIter, Iter endIter);

};