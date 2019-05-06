#include <gmock/gmock.h>
#include <string>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <vector>

using std::vector;
using namespace testing;

namespace transmission_interface
{
// Floating-point value comparison threshold
const double EPS = 1e-6;

class PositionOnlyActuatorData : public ActuatorDataContainer<PositionActuatorData>
{
  public:
    bool empty() const override
    {
        return position.empty();
    }

    bool hasSize(std::size_t size) const override
    {
        return (not position.empty() and position.size() == size);
    }

    bool valid() const override
    {
        return hasValidPointers(position);
    }
};

class DummyHandle : public TransmissionHandle<PositionOnlyActuatorData>
{
  public:
    DummyHandle(const std::string& name, Transmission* transmission, const PositionOnlyActuatorData& actuator_data,
                const JointData& joint_data)
      : TransmissionHandle(name, transmission, actuator_data, joint_data)
    {
    }
};

TEST(HandlePreconditionsTest, ValidHandle)
{
    double val = 0.0;
    vector<double*> good_vec(1, &val);
    SimpleTransmission trans(1.0);

    {
        PositionOnlyActuatorData a_data;
        JointData j_data;
        a_data.position = good_vec;
        EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
    }
    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        JointData j_data;
        j_data.position = good_vec;
        EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
    }
    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        JointData j_data;
        j_data.velocity = good_vec;
        EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
    }
    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        JointData j_data;
        j_data.effort = good_vec;
        EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
    }
    {
        PositionOnlyActuatorData a_data;
        JointData j_data;
        j_data.position = good_vec;
        EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
    }
    {
        PositionOnlyActuatorData a_data;
        JointData j_data;
        j_data.velocity = good_vec;
        EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
    }
    {
        PositionOnlyActuatorData a_data;
        JointData j_data;
        j_data.effort = good_vec;
        EXPECT_THROW(DummyHandle("trans", &trans, a_data, j_data), TransmissionInterfaceException);
    }
    {
        PositionOnlyActuatorData a_data;
        JointData j_data;
        a_data.position = good_vec;
        j_data.position = good_vec;
        j_data.velocity = good_vec;
        j_data.effort = good_vec;
        EXPECT_NO_THROW(DummyHandle("trans", &trans, a_data, j_data));
    }
}

}  // namespace transmission_interface

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
