#include <gtest/gtest.h>
#include "nasa_robodyn_mechanisms_core/EmbeddedSmoother.h"
#include <auto_ptr.h>
#include <fstream>

class EmbeddedSmootherTest : public ::testing::Test {
protected:
    virtual void SetUp()
    {
        settings.timestep = 0.03;
        settings.accGain = 1000;
        settings.maxAcc = 2;
        settings.minVel = 0.01;
        smoother.reset(new EmbeddedSmoother(settings));
    }

    virtual void TearDown()
    {
    }

    std::auto_ptr<EmbeddedSmoother> smoother;
    EmbeddedSmoother::Settings settings;
};

TEST_F(EmbeddedSmootherTest, reset)
{
    double ret;
    ret = smoother->reset(.5, 0.);
    EXPECT_EQ(.5, ret);

    ret = smoother->reset(1., 0.);
    EXPECT_EQ(1., ret);
}

TEST_F(EmbeddedSmootherTest, update)
{
    double ret, ret2;
    ret = smoother->reset(.5, 0.);
    ret = smoother->update(1., 0.);

    // should be moving towards goal despite 0 velocity
    EXPECT_GT(ret, .5);

    // shouldn't be able to achieve a velocity of 1 in one timestep
    ret = smoother->reset(.5, 0.);
    ret = smoother->update(1., 1.);
    EXPECT_LT(ret, .5 + settings.timestep * 1.);

    // should continually make progress with minimal overshoot
    ret = smoother->reset(.5, 0.);
    unsigned int it = 0;
    do
    {
        ret2 = ret;
        ++it;
        ret = smoother->update(1., 1.);
        EXPECT_GE(ret, ret2);

    } while (ret2 != ret && it < 10000);

    EXPECT_LT(it, 10000);
    EXPECT_GE(ret, 1.);
    EXPECT_NEAR(ret - 1., 0.5 * 1. / settings.maxAcc, 0.01);
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
