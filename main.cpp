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

    /// visualize the original clip
    glutInit(&argc, argv);
    test.initWindow(1280, 700, "Biped");
    glutMainLoop();

    return 0;
}