#ifndef DETERMINISTIC_CONTROLLER_H
#define DETERMINISTIC_CONTROLLER_H

#include <dart/dart.hpp>
#include <dart/utils/utils.hpp>
#include <dart/gui/gui.hpp>

class Controller
{
public:
    /// Constructor
    Controller(const dart::dynamics::SkeletonPtr& biped)
        : mBiped(biped),
          mSpeed(0.0)
    {
        int nDofs = static_cast<int>(mBiped->getNumDofs());

        mForces = Eigen::VectorXd::Zero(nDofs);

        mKp = Eigen::MatrixXd::Identity(nDofs, nDofs);
        mKd = Eigen::MatrixXd::Identity(nDofs, nDofs);

        for(std::size_t i = 0; i < 6; ++i)
        {
            mKp(i, i) = 0.0;
            mKd(i, i) = 0.0;
        }

        for(std::size_t i = 6; i < mBiped->getNumDofs(); ++i)
        {
            mKp(i, i) = 300;
            mKd(i, i) = 30;
        }

        setTargetPositions(mBiped->getPositions());
    }

    /// Reset the desired dof position to the current position
    void setTargetPositions(const Eigen::VectorXd& pose)
    {
        mTargetPositions = pose;
    }

    /// Clear commanding forces
    void clearForces()
    {
        mForces.setZero();
    }

    /// Add commanding forces from PD controllers (Lesson 2 Answer)
    void addPDForces()
    {
        Eigen::VectorXd q = mBiped->getPositions();
        Eigen::VectorXd dq = mBiped->getVelocities();

        Eigen::VectorXd p = -mKp * (q - mTargetPositions);
        Eigen::VectorXd d = -mKd * dq;

        std::cout << "addPDForces" << std::endl;
        std::cout << "---------------------" << std::endl;
        std::cout << "q" << std::endl;
        for(int i = 0; i < q.size(); i++)
        {
            std::cout << q[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "mTargetPositions" << std::endl;
        for(int i = 0; i < mTargetPositions.size(); i++)
        {
            std::cout << mTargetPositions[i] << " ";
        }
        std::cout << std::endl;

        std::cout << "dq" << std::endl;
        for(int i = 0; i < dq.size(); i++)
        {
            std::cout << dq[i] << " ";
        }
        std::cout << std::endl;

        mForces += p + d;

        std::cout << "mForces" << std::endl;
        for(int i = 0; i < mForces.size(); i++)
        {
            std::cout << mForces[i] << " ";
        }
        std::cout << std::endl;

        mBiped->setForces(mForces);
    }

    /// Add commanind forces from Stable-PD controllers (Lesson 3 Answer)
    void addSPDForces()
    {
        Eigen::VectorXd q = mBiped->getPositions();
        Eigen::VectorXd dq = mBiped->getVelocities();

        Eigen::MatrixXd invM = (mBiped->getMassMatrix()
                                + mKd * mBiped->getTimeStep()).inverse();
        Eigen::VectorXd p =
            -mKp * (q + dq * mBiped->getTimeStep() - mTargetPositions);
        Eigen::VectorXd d = -mKd * dq;
        Eigen::VectorXd qddot =
            invM * (-mBiped->getCoriolisAndGravityForces()
                    + p + d + mBiped->getConstraintForces());

        mForces += p + d - mKd * qddot * mBiped->getTimeStep();
        // clipForces();
        mBiped->setForces(mForces);
    }

    Eigen::VectorXd getForces()
    {
        return mForces;
    }

    /// add commanding forces from ankle strategy (Lesson 4 Answer)
    void addAnkleStrategyForces()
    {
        Eigen::Vector3d COM = mBiped->getCOM();
        // Approximated center of pressure in sagittal axis
        Eigen::Vector3d offset(0.05, 0, 0);
        Eigen::Vector3d COP = mBiped->getBodyNode("h_heel_left")->
            getTransform() * offset;
        double diff = COM[0] - COP[0];

        Eigen::Vector3d dCOM = mBiped->getCOMLinearVelocity();
        Eigen::Vector3d dCOP =  mBiped->getBodyNode("h_heel_left")->
            getLinearVelocity(offset);
        double dDiff = dCOM[0] - dCOP[0];

        int lHeelIndex = mBiped->getDof("j_heel_left_1")->getIndexInSkeleton();
        int rHeelIndex = mBiped->getDof("j_heel_right_1")->getIndexInSkeleton();
        int lToeIndex = mBiped->getDof("j_toe_left")->getIndexInSkeleton();
        int rToeIndex = mBiped->getDof("j_toe_right")->getIndexInSkeleton();
        if(diff < 0.1 && diff >= 0.0)
        {
            // Feedback rule for recovering forward push
            double k1 = 200.0;
            double k2 = 100.0;
            double kd = 10;
            mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
            mForces[lToeIndex] += -k2 * diff - kd * dDiff;
            mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
            mForces[rToeIndex] += -k2 * diff - kd * dDiff;
        }
        else if(diff > -0.2 && diff < -0.05)
        {
            // Feedback rule for recovering backward push
            double k1 = 2000.0;
            double k2 = 100.0;
            double kd = 100;
            mForces[lHeelIndex] += -k1 * diff - kd * dDiff;
            mForces[lToeIndex] += -k2 * diff - kd * dDiff;
            mForces[rHeelIndex] += -k1 * diff - kd * dDiff;
            mForces[rToeIndex] += -k2 * diff - kd * dDiff;
        }
        mBiped->setForces(mForces);
    }

    // Send velocity commands on wheel actuators (Lesson 6 Answer)
    void setWheelCommands()
    {
        int wheelFirstIndex = (int)mBiped->getDof("joint_front_left_1")->getIndexInSkeleton();
        for (std::size_t i = (int)wheelFirstIndex; i < mBiped->getNumDofs(); ++i)
        {
            mKp(i, i) = 0.0;
            mKd(i, i) = 0.0;
        }

        size_t index1 = mBiped->getDof("joint_front_left_2")->getIndexInSkeleton();
        size_t index2 = mBiped->getDof("joint_front_right_2")->getIndexInSkeleton();
        size_t index3 = mBiped->getDof("joint_back_left")->getIndexInSkeleton();
        size_t index4 = mBiped->getDof("joint_back_right")->getIndexInSkeleton();
        mBiped->setCommand(index1, mSpeed);
        mBiped->setCommand(index2, mSpeed);
        mBiped->setCommand(index3, mSpeed);
        mBiped->setCommand(index4, mSpeed);
    }

    void changeWheelSpeed(double increment)
    {
        mSpeed += increment;
        std::cout << "wheel speed = " << mSpeed << std::endl;
    }

    void setSkeleton(dart::dynamics::SkeletonPtr _biped)
    {
        mBiped = _biped;
    }

protected:
    /// The biped Skeleton that we will be controlling
    dart::dynamics::SkeletonPtr mBiped;

    /// Joint forces for the biped (output of the Controller)
    Eigen::VectorXd mForces;

    /// Control gains for the proportional error terms in the PD controller
    Eigen::MatrixXd mKp;

    /// Control gains for the derivative error terms in the PD controller
    Eigen::MatrixXd mKd;

    /// Target positions for the PD controllers
    Eigen::VectorXd mTargetPositions;

    /// For velocity actuator: Current speed of the skateboard
    double mSpeed;
};


#endif //SPACETIME_CONTROLLER_H
