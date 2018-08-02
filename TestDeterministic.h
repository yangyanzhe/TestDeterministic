#ifndef DETERMINISTIC_TESTDETERMINISTIC_H
#define DETERMINISTIC_TESTDETERMINISTIC_H

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/gui.hpp>
#include <fstream>
#include <vector>
#include "Controller.h"

class State
{
public:
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;

    State() = default;

    State(int nDof)
    {
        pos = Eigen::VectorXd::Zero(nDof);
        vel = Eigen::VectorXd::Zero(nDof);
    }

    State(Eigen::VectorXd _pos, Eigen::VectorXd _vel)
        : pos(_pos), vel(_vel)
    {

    }
};

class TestDeterministic : public dart::gui::SimWindow
{
private:
    dart::simulation::WorldPtr mWorld;
    dart::dynamics::SkeletonPtr mBiped;
    std::shared_ptr<Controller> mController;

    double fps = 30.0;
    std::vector<Eigen::VectorXd> mPoses;
    std::vector<Eigen::VectorXd> mVels;
    std::vector<Eigen::VectorXd> mTargets;

    int nSample = 0;

private:
    dart::dynamics::SkeletonPtr loadBiped();
    dart::dynamics::SkeletonPtr createFloor();
    State forward(State & start, Eigen::VectorXd & target);
    std::vector<Eigen::VectorXd> loadArray(std::ifstream & infile, int nRows, int nCols);

public:
    TestDeterministic()
    {
        InitEnv();
    }

    void InitEnv();
    void LoadTestData(const char* filename);
    void RunTest();

    void InitMotion();
    void timeStepping() override;
};

#endif //DETERMINISTIC_TESTDETERMINISTIC_H
