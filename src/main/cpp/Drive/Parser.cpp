#include "Drive/Parser.h"
#include <fstream>
#include <fmt/core.h>
#include <cmath>

std::string Parser::getFile(std::filesystem::path path) {
  std::ifstream file(path);
  if (!file) {
    fmt::print("Error: Could not open file \"{}\" from directory \"{}\"\n", path.string(), std::filesystem::current_path().string());
    return "";
  }

  std::string str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

  // Remove DOS line endings.
  std::string new_str;
  for (const char& c : str) {
    if (c != '\r') {
      new_str.push_back(c);
    }
  }

  return new_str;
}

std::ptrdiff_t Parser::countWhile(Iter currIter, Iter endIter, ConditionFunc condFunc) {
  std::ptrdiff_t count = 0;
  while (currIter != endIter && condFunc(*currIter)) {
    ++currIter;
    ++count;
  }
  return count;
}

std::string Parser::parseWhile(Iter& currIter, Iter endIter, ConditionFunc condFunc) {
  std::ptrdiff_t count = countWhile(currIter, endIter, condFunc);
  std::string str(currIter, currIter + count);
  currIter += count;
  return str;
}

std::ptrdiff_t Parser::countUntil(Iter currIter, Iter endIter, std::string chars) {
  return countWhile(currIter, endIter, [&chars](char c) {
    return chars.find(c) == std::string::npos;
  });
}

std::string Parser::parseUntil(Iter& currIter, Iter endIter, std::string chars) {
  std::ptrdiff_t count = countUntil(currIter, endIter, chars);
  std::string str(currIter, currIter + count);
  currIter += count;
  return str;
}

double Parser::parseNumber(Iter& currIter, Iter endIter) {
  // Handle + and -
  int sign = 1;
  if (*currIter == '-') {
    sign = -1;
    ++currIter;
  }
  else if (*currIter == '+') {
    ++currIter;
  }

  // Parse the integer part.
  std::string intStr = parseWhile(currIter, endIter, [](char c) {
    return std::isdigit(c);
  });

  bool do_more = *currIter == '.' || *currIter == 'e';
  
  try
  {
    if (currIter == endIter || !do_more) {
      return std::stoi(intStr) * sign;
    }
  }
  catch(const std::exception& e)
  {
    printf("Error line 76: %s \n", e.what());
  }
  
  double num = 0;
  try
  {
    num = std::stoi(intStr);
  }
  catch(const std::exception& e)
  {
    printf("Error line 87: %s \n", e.what());
  }


  // Parse the decimal part.
  if (*currIter == '.') {
    ++currIter;
    std::string decStr = parseWhile(currIter, endIter, [](char c) {
      return std::isdigit(c);
    });

    num = std::stod(fmt::format("{}.{}", intStr, decStr)) * sign;
  }

  // Parse the exponent part.
  if (*currIter == 'e' || *currIter == 'E') {
    ++currIter;
    int expSign = 1;
    if (*currIter == '-') {
      expSign = -1;
      ++currIter;
    }
    else if (*currIter == '+') {
      ++currIter;
    }

    std::string expStr = parseWhile(currIter, endIter, [](char c) {
      return std::isdigit(c);
    });

    num *= std::pow(10, std::stoi(expStr) * expSign);
  }

  return num;
}