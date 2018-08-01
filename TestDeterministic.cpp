#include <iostream>
#include <vector>
#include <regex>
#include <fstream>
#include <boost/filesystem.hpp>

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/gui.hpp>

#include "TestDeterministic.h"


using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::gui;
using namespace dart::utils;
using namespace dart::math;
using namespace std;

SkeletonPtr TestDeterministic::loadBiped()
{
    // Create the world with a skeleton
    auto path = boost::filesystem::current_path();
    string bipedSkeletonFile = path.string() + "/biped.skel";

    WorldPtr world = SkelParser::readWorld(bipedSkeletonFile.c_str());
    if(access("biped.skel", F_OK ) == -1)
    {
        cout << "Cannot access biped.skel. Check if exists." << endl;
    }
    if(world == nullptr)
    {
        throw "Error in loading the world";
    }

    SkeletonPtr biped = world->getSkeleton("biped");

    // Set joint limits
    for(std::size_t i = 0; i < biped->getNumJoints(); ++i)
        biped->getJoint(i)->setPositionLimitEnforced(true);

    // Enable self collision check but ignore adjacent bodies
    biped->enableSelfCollisionCheck();
    biped->disableAdjacentBodyCheck();

    return biped;
}

SkeletonPtr TestDeterministic::createFloor()
{
    SkeletonPtr floor = Skeleton::create("floor");

    // Give the floor a body
    BodyNodePtr body =
        floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;

    // Give the body a shape
    double floor_width = 10.0;
    double floor_height = 0.01;
    std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_height, floor_width)));
    auto shapeNode
        = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(dart::Color::Black());

    // Put the body into position
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    //tf.translation() = Eigen::Vector3d(0.0, -1.0, 0.0);
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return floor;
}

void TestDeterministic::InitEnv()
{
    SkeletonPtr floor = createFloor();
    mBiped = loadBiped();
    mWorld = std::make_shared<World>();
    mWorld->setGravity(Eigen::Vector3d(0.0, -9.81, 0.0));
    if (dart::collision::CollisionDetector::getFactory()->canCreate("bullet"))
    {
        mWorld->getConstraintSolver()->setCollisionDetector(
            dart::collision::CollisionDetector::getFactory()->create("bullet"));
    }
    mWorld->addSkeleton(floor);
    mWorld->addSkeleton(mBiped);
    mController = std::make_shared<Controller>(mBiped);
}

std::vector<Eigen::VectorXd> TestDeterministic::loadArray(ifstream & infile, int nRows, int nCols)
{
    std::vector<Eigen::VectorXd> rs;
    if(nCols == 0 || nRows == 0)
        return rs;

    string line;
    for(int i = 0; i < nRows; i++)
    {
        getline(infile, line);
        Eigen::VectorXd tmp = Eigen::VectorXd::Zero(nCols);

        char * dup = strdup(line.c_str());
        char * token = strtok(dup, " ");
        int joint_id = 0;
        while(token != NULL){
            if(joint_id >= nCols)
                break;
            tmp[joint_id] = stod(token);
            token = strtok(NULL, " ");
            joint_id ++;
        }
        free(dup);

        rs.push_back(std::move(tmp));
    }
    return rs;
}

void TestDeterministic::LoadTestData(const char* filename)
{
    ifstream infile(filename);
    if(!infile.is_open())
        cout << "Error loading the file" << filename << ". Check if exists." << endl;

    string line;
    int nDof = 0;
    while(getline(infile, line))
    {
        std::regex regSample("nSample: (0|[1-9][0-9]*)");
        std::regex regDof("nDof: (0|[1-9][0-9]*)");

        std::smatch match;
        if (std::regex_search(line, match, regSample) && match.size() > 1)
        {
            string match_str = match[1].str();
            nSample = atoi(match_str.c_str());
            continue;
        }
        if (std::regex_search(line, match, regDof) && match.size() > 1)
        {
            string match_str = match[1].str();
            nDof = atoi(match_str.c_str());
            continue;
        }

        if(strcmp(line.c_str(), "pos:") == 0)
        {
            mPoses = loadArray(infile, nSample, nDof);
        }
        else if(strcmp(line.c_str(), "vel:") == 0)
        {
            mVels = loadArray(infile, nSample, nDof);
        }
        else if(strcmp(line.c_str(), "target:") == 0)
        {
            mTargets = loadArray(infile, nSample, nDof);
        }
    }

    cout << "Finish loading test data" << endl;
}

State TestDeterministic::forward(State & start, Eigen::VectorXd & target)
{
    State end;

    mWorld->reset();
    mBiped->setPositions(start.pos);
    mBiped->setVelocities(start.vel);
    mController->setTargetPositions(target);

    int numStep = 100;
    for(int i = 0; i < numStep; i++)
    {
        mController->clearForces();
        mController->addSPDForces();
        mWorld->step();
    }

    end.pos = mBiped->getPositions();
    end.vel = mBiped->getVelocities();

    return end;
}

void TestDeterministic::RunTest()
{
    int nRuns = 10;
    bool passAll = true;

    for(int i = 0; i < nSample; i++)
    {
        std::vector<State> testResults;
        for(int j = 0; j < nRuns; j++)
        {
            if(i >= mPoses.size())
                throw "Index out of range.";

            State start(mPoses[i], mVels[i]);
            State end = forward(start, mTargets[i]);
            testResults.push_back(std::move(end));
        }

        // test whether the results are deterministic
        // if success, test results should be identical
        bool fail = false;
        for(int j = 0; j < nRuns; j++)
        {
            for(int k = j+1; k < nRuns; k++)
            {
                double distance = (testResults[j].pos - testResults[k].pos).norm();
                if(distance > 1e-4)
                {
                    fail = true;
                    break;
                }
            }
            if(fail)
                break;
        }

        if(fail)
        {
            passAll = false;
            cout << "Test " << i << " fails." << endl;
        }
    }

    if(passAll)
        cout << "pass all tests." << endl;
}