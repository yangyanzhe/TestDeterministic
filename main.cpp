#include <iostream>
#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/gui.hpp>

#include "TestDeterministic.h"


int main(int argc, char* argv[])
{
    const char* dataFile = "data.txt";

    TestDeterministic test;
    test.LoadTestData(dataFile);
    test.RunTest();

    return 0;
}