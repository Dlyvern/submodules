#include "MagneticStreamer.h"

MagneticStreamer::MagneticStreamer(const std::string &name, std::unique_ptr<Robot> robot) : Module(name), m_Robot{std::move(robot)}
{
    m_FakeCoordinate = 0;
}

void MagneticStreamer::FindScanner()
{
    while(m_MagneticScanner == nullptr)
    {
        m_MagneticScanner = m_Robot->FindDevice("magneticscanner");
    }

    Log("Found scanner", INFO_LEVEL_LOG);
}

void MagneticStreamer::Work()
{
    BeforeRun();
    SetRunning(true);

    while(IsRunning() && IsActive())
    {
        while(IsActive() && m_MagneticScanner->IsRunning()))
        {

        }
    }
}

std::vector<std::vector<uint8_t>> MagneticStreamer::CreateFakeChunk()
{
    auto random = [](int min, int max)
    {
        std::random_device dev;
        std::mt19937 rng(dev());
        std::uniform_int_distribution<std::mt19937::result_type> dist6(min,max);

        return dist6(rng);
    };

    const uint16_t size = 500;

    std::vector<std::vector<uint32_t>>points;

    points.reserve(size); //For better performance

    for(int i = 0; i < size; ++i)
    {
        uint32_t point = random(2000, 3000);
        uint32_t point2 = point + random(0, 10);

        points.emplace_back(std::vector<uint32_t>{point, point2});
    }

    std::vector<uint32_t> coordinate = {m_FakeCoordinate, m_FakeCoordinate + 1};

//    auto chunk = {coordinate, points};
//
//    return chunk;
}

bool MagneticStreamer::Enable()
{
    IsActive();
    if(IsActive())
    {
        Log("Magnetic scanner stream is enabled already!");
        return false;
    }

    SetActive(true);
    return true;
}

bool MagneticStreamer::Disable()
{
    if(!IsActive())
    {
        Log("Magnetic scanner stream is disabled already!");
        return false;
    }

    SetActive(false);
    return true;
}

void MagneticStreamer::BeforeRun()
{
    FindScanner();
}

