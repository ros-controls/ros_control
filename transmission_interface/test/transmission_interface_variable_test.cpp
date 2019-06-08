#include <gmock/gmock.h>
#include <string>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <vector>
#include <sstream>

using namespace testing;

namespace transmission_interface
{
template <typename T>
std::vector<T> operator+(const std::vector<T>& v1, const std::vector<T>& v2)
{
    std::vector<T> result;
    result.insert(result.end(), v1.begin(), v1.end());
    result.insert(result.end(), v2.begin(), v2.end());
    return result;
}

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

    size_t size() const
    {
        return position.size();
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

struct TransmissionTestParams
{
    PositionOnlyActuatorData actuator_data;
    JointData joint_data;
    bool should_throw;
};

std::string PrintToString(const TransmissionTestParams& param)
{
    std::stringstream ss;
    ss << "ActuatorDataSize: " << param.actuator_data.size()
       << " JointDataPositionSize: " << param.joint_data.position.size()
       << " VelocitySize: " << param.joint_data.velocity.size() << " EffortSize: " << param.joint_data.effort.size()
       << " ShouldThrow: " << (param.should_throw ? "Yes" : "No");
    return ss.str();
}

class VariableTransmissionInterfaceTest : public TestWithParam<TransmissionTestParams>
{
  public:
    VariableTransmissionInterfaceTest() : trans(1.0)
    {
    }

    SimpleTransmission trans;
};

TEST_P(VariableTransmissionInterfaceTest, transmissionHandleInitialization)
{
    if (GetParam().should_throw)
    {
        EXPECT_THROW(DummyHandle("trans", &trans, GetParam().actuator_data, GetParam().joint_data),
                     TransmissionInterfaceException);
    }
    else
    {
        EXPECT_NO_THROW(DummyHandle("trans", &trans, GetParam().actuator_data, GetParam().joint_data));
    }
}

std::vector<TransmissionTestParams> makeTransmissionHandleTestParams(const int num_dof)
{
    std::vector<TransmissionTestParams> result;
    constexpr bool SHOULD_THROW = true;
    constexpr bool SHOULD_NOT_THROW = false;

    double val = 0.0;
    std::vector<double*> good_vec(num_dof, &val);

    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        result.push_back({ a_data, JointData(), SHOULD_NOT_THROW });
    }
    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        JointData j_data;
        j_data.position = good_vec;
        result.push_back({ a_data, j_data, SHOULD_NOT_THROW });
    }
    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        JointData j_data;
        j_data.velocity = good_vec;
        result.push_back({ a_data, j_data, SHOULD_NOT_THROW });
    }
    {
        PositionOnlyActuatorData a_data;
        a_data.position = good_vec;
        JointData j_data;
        j_data.effort = good_vec;
        result.push_back({ a_data, j_data, SHOULD_NOT_THROW });
    }
    {
        JointData j_data;
        j_data.position = good_vec;
        result.push_back({ PositionOnlyActuatorData(), j_data, SHOULD_THROW });
    }
    {
        JointData j_data;
        j_data.velocity = good_vec;
        result.push_back({ PositionOnlyActuatorData(), j_data, SHOULD_THROW });
    }
    {
        JointData j_data;
        j_data.effort = good_vec;
        result.push_back({ PositionOnlyActuatorData(), j_data, SHOULD_THROW });
    }
    {
        PositionOnlyActuatorData a_data;
        JointData j_data;
        a_data.position = good_vec;
        j_data.position = good_vec;
        j_data.velocity = good_vec;
        j_data.effort = good_vec;
        result.push_back({ a_data, j_data, SHOULD_NOT_THROW });
    }

    return result;
}

// SimpleTransmission only handles one joint
INSTANTIATE_TEST_CASE_P(TransmissionHandleInitialization, VariableTransmissionInterfaceTest,
                        ValuesIn(makeTransmissionHandleTestParams(1)));

}  // namespace transmission_interface

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
