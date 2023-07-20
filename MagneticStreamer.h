#include "Robot.h"
#include "MagneticScanner.h"
#include "Module.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <random>

class MagneticStreamer : public Module
{
private:
    static uint32_t m_FakeCoordinate;
    std::unique_ptr<Robot>m_Robot;
    std::shared_ptr<MagneticScanner>m_MagneticScanner{nullptr};
public:

    MagneticStreamer(const std::string& name, std::unique_ptr<Robot>robot);

    void FindScanner();

    void Work() override;

    void BeforeRun() override;

    std::string Enable(RunParameters& runParameters) override;

    std::string Disable(RunParameters& runParameters) override;

    std::vector<std::vector<uint8_t>> CreateFakeChunk();
};
