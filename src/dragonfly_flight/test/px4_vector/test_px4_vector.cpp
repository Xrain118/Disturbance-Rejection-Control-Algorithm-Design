#include <gtest/gtest.h>
#include <PositionControl.hpp>
#include <ControlAllocationDragonfly.hpp>
#include <geo/geo.h>
#include <px4_platform_common/init.h>
#include <lib/parameters/param.h>
#include "bridge_fixture.hpp"

using namespace matrix;

static PositionControl controller(bool vector_mode)
{
    PositionControl c;
    c.setVectorMode(vector_mode, 19.62f);
    c.setPositionGains(Vector3f(1.f,1.f,1.f));
    c.setVelocityGains(Vector3f(2.f,2.f,2.f), Vector3f(.3f,.3f,.3f), Vector3f());
    c.setVelocityLimits(2.f,1.f,1.f); c.setThrustLimits(.01f,1.f);
    c.setHorizontalThrustMargin(.3f); c.setTiltLimit(1.f); c.setHoverThrust(.5f);
    c.setState(PositionControlStates{});
    return c;
}

TEST(NativeVector, BodyThrustUsesCurrentAttitudeAndWorldForceIsPreserved)
{
    auto c=controller(true);
    const Quatf q(Eulerf(.3f,-.2f,.7f)); c.setBodyAttitude(q);
    auto sp=PositionControl::empty_trajectory_setpoint;
    Vector3f(2.f,0.f,0.f).copyTo(sp.acceleration);sp.yaw=.8f;
    c.setInputSetpoint(sp);ASSERT_TRUE(c.update(.01f));
    vehicle_attitude_setpoint_s out{};c.getAttitudeSetpoint(out);
    const Vector3f ned=Dcmf(q)*Vector3f(out.thrust_body);
    EXPECT_NEAR(ned(0),2.f/19.62f,1e-6f);
    EXPECT_NEAR(ned(2),-CONSTANTS_ONE_G/19.62f,1e-6f);
    EXPECT_NEAR(Eulerf(Quatf(out.q_d)).phi(),0.f,1e-6f);
    EXPECT_NEAR(Eulerf(Quatf(out.q_d)).theta(),0.f,1e-6f);
    EXPECT_NEAR(Eulerf(Quatf(out.q_d)).psi(),.8f,1e-6f);
}

TEST(NativeVector, HTECannotChangeVectorThrustOrIntegral)
{
    auto c=controller(true);auto sp=PositionControl::empty_trajectory_setpoint;
    Vector3f().copyTo(sp.position);c.setInputSetpoint(sp);ASSERT_TRUE(c.update(.01f));
    const auto before=c.getThrustNED();c.updateHoverThrust(.8f);
    c.setInputSetpoint(sp);ASSERT_TRUE(c.update(.01f));
    EXPECT_EQ(before,c.getThrustNED());
}

TEST(NativeVector, InvalidAttitudeIsRejectedAndNonunitAttitudeIsNormalized)
{
    auto c=controller(true);auto sp=PositionControl::empty_trajectory_setpoint;
    Vector3f().copyTo(sp.position);c.setInputSetpoint(sp);
    c.setBodyAttitude(Quatf(0.f,0.f,0.f,0.f));EXPECT_FALSE(c.update(.01f));
    c.setBodyAttitude(Quatf(2.f,0.f,0.f,0.f));EXPECT_TRUE(c.update(.01f));
}

TEST(NativeVector, StandardMultirotorStillTiltsForHorizontalForce)
{
    auto c=controller(false);auto sp=PositionControl::empty_trajectory_setpoint;
    Vector3f(2.f,0.f,0.f).copyTo(sp.acceleration);c.setInputSetpoint(sp);ASSERT_TRUE(c.update(.01f));
    vehicle_attitude_setpoint_s out{};c.getAttitudeSetpoint(out);
    EXPECT_GT(fabsf(Eulerf(Quatf(out.q_d)).theta()),.05f);
    EXPECT_FLOAT_EQ(out.thrust_body[0],0.f);
    EXPECT_FLOAT_EQ(out.thrust_body[1],0.f);
}

TEST(NativeVector, HoverGeometryAndMotorPairsProduceNoOriginMoment)
{
    DragonflyAllocation solver;
    DragonflyAllocation::Wrench desired;desired(2)=solver.geometry.mass*9.81f;
    ASSERT_TRUE(solver.solve(desired,.01f));
    const auto result=solver.wrench(solver.state);
    EXPECT_NEAR(result(2),desired(2),1e-3f);
    for(int i=0;i<6;++i) { EXPECT_NEAR(solver.state(i),desired(2)/6.f,1e-3f); }
    for(int i=3;i<6;++i) { EXPECT_NEAR(result(i),0.f,1e-3f); }
}

TEST(NativeVector, TiltGeometryIncludesSharedAxesAndMovingMotorPositions)
{
    DragonflyAllocation solver;DragonflyAllocation::State state;
    state(0)=7.f;state(6)=.2f;state(7)=-.3f;
    const auto result=solver.wrench(state);
    // First unit has Rz=I, Rx(.2)*Ry(-.3), so local +Z rotates analytically.
    const Vector3f axis(sinf(-.3f),-sinf(.2f)*cosf(-.3f),cosf(.2f)*cosf(-.3f));
    const Dcmf rotation=Dcmf(Eulerf(.2f,0.f,0.f))*Dcmf(Eulerf(0.f,-.3f,0.f));
    const Vector3f p=Vector3f(solver.geometry.radius,0.f,solver.geometry.height)+rotation*Vector3f(0.f,.18f,.038f);
    const Vector3f moment=7.f*(p.cross(axis)-.02f*axis);
    for(int i=0;i<3;++i) { EXPECT_NEAR(result(i),7.f*axis(i),1e-5f);EXPECT_NEAR(result(i+3),moment(i),1e-5f); }
}

TEST(NativeVector, FeasibleWrenchConvergesAndServoRateIsBounded)
{
    DragonflyAllocation solver;DragonflyAllocation::Wrench desired;
    desired(0)=2.f;desired(1)=-1.f;desired(2)=45.f;desired(3)=.3f;desired(4)=-.2f;desired(5)=.1f;
    for(int k=0;k<100;++k) {
        const auto before=solver.state;const bool initialized=solver.initialized;
        ASSERT_TRUE(solver.solve(desired,.01f));
        if(initialized) { for(int i=6;i<12;++i) { EXPECT_LE(fabsf(solver.state(i)-before(i)),.00501f); } }
    }
    EXPECT_LT(DragonflyAllocation::Wrench(solver.wrench(solver.state)-desired).norm(),.03f);
}

TEST(NativeVector, UnreachableWrenchKeepsEveryActuatorFiniteAndBounded)
{
    DragonflyAllocation solver;DragonflyAllocation::Wrench desired;
    desired(0)=500.f;desired(2)=1000.f;desired(3)=200.f;
    ASSERT_TRUE(solver.solve(desired,.01f));
    for(int i=0;i<6;++i) { EXPECT_GE(solver.state(i),0.f);EXPECT_LE(solver.state(i),solver.geometry.max_thrust); }
    for(int i=6;i<12;++i) { EXPECT_LE(fabsf(solver.state(i)),solver.geometry.tilt_limit); }
    EXPECT_TRUE(solver.state.isAllFinite());
    desired(1)=NAN;EXPECT_FALSE(solver.solve(desired,.01f));
}

TEST(NativeVector, PublishedAllocationResidualAccountsForMotorSpeedSquaredAndServoAngles)
{
    // Equal thrust is a valid expectation only for the symmetric CoM fixture.
    const float zero=0.f;
    param_set_no_notification(param_find("DF_COM_X"),&zero);
    param_set_no_notification(param_find("DF_COM_Y"),&zero);
    param_set_no_notification(param_find("DF_COM_Z"),&zero);
    ControlAllocationDragonfly allocator;
    Matrix<float,6,16> effectiveness;Vector<float,16> trim;
    allocator.setEffectivenessMatrix(effectiveness,trim,trim,12,false);
    Vector<float,16> lower,upper;upper.setAll(1.f);for(int i=6;i<12;++i) { lower(i)=-1.f; }
    allocator.setActuatorMin(lower);allocator.setActuatorMax(upper);
    Vector<float,6> desired;desired(5)=-9.81f/19.62f;
    allocator.setControlSetpoint(desired);
    for(int k=0;k<20;++k) { allocator.allocate(); }
    const auto out=allocator.getActuatorSetpoint();
    for(int i=0;i<6;++i) { EXPECT_NEAR(out(i),sqrtf(.5f),.002f); }
    const auto achieved=allocator.getAllocatedControl();
    EXPECT_LT(DragonflyAllocation::Wrench(achieved-desired).norm(),.001f);
    auto clipped=out;clipped(0)=0.f;allocator.setActuatorSetpoint(clipped);
    EXPECT_GT(DragonflyAllocation::Wrench(allocator.getAllocatedControl()-desired).norm(),.01f);
}

TEST(NativeVector, PythonPhysicalBridgeMatchesCppAllocatorWithOffsetAndFullInertia)
{
    const char *names[]={"DF_MASS","DF_ACC","DF_RATE","DF_FMAX","DF_COM_X","DF_COM_Y","DF_COM_Z",
        "DF_IXX","DF_IYY","DF_IZZ","DF_IXY","DF_IXZ","DF_IYZ"};
    const float values[]={5.415f,19.62f,4.f/.15f,5.415f*19.62f/6.f,.02f,.03f,-.14f,.6f,.7f,.8f,.01f,.02f,.03f};
    float saved[13];
    for(int i=0;i<13;++i) { param_get(param_find(names[i]),&saved[i]);param_set_no_notification(param_find(names[i]),&values[i]); }
    ControlAllocationDragonfly allocator;
    Matrix<float,6,16> effectiveness;Vector<float,16> trim,lower,upper;upper.setAll(1.f);
    allocator.setEffectivenessMatrix(effectiveness,trim,trim,12,false);
    for(int i=6;i<12;++i) { lower(i)=-1.f; }
    allocator.setActuatorMin(lower);allocator.setActuatorMax(upper);
    Vector<float,6> desired;
    for(int i=0;i<6;++i) { desired(i)=bridge_control[i]; }
    allocator.setControlSetpoint(desired);
    for(int k=0;k<500;++k) { allocator.allocate(); }
    const auto out=allocator.getActuatorSetpoint();
    DragonflyAllocation physical;physical.geometry.mass=5.415f;physical.geometry.max_thrust=values[3];
    DragonflyAllocation::State state;
    for(int i=0;i<6;++i) { state(i)=out(i)*out(i)*values[3];state(i+6)=out(i+6)*physical.geometry.tilt_limit; }
    const auto wrench=physical.wrench(state);
    for(int i=0;i<6;++i) { EXPECT_NEAR(wrench(i),bridge_physical[i],.03f); }
    const auto achieved=allocator.getAllocatedControl();
    for(int i=0;i<6;++i) { EXPECT_NEAR(achieved(i),desired(i),.002f); }
    for(int i=0;i<13;++i) { param_set_no_notification(param_find(names[i]),&saved[i]); }
}

int main(int argc,char **argv)
{
    testing::InitGoogleTest(&argc,argv);px4_platform_init();
    // This host harness has no simulated clock or persistent parameter file.
    param_control_autosave(false);
    return RUN_ALL_TESTS();
}
