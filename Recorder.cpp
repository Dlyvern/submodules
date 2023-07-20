#include "Recorder.h"

#include <utility>

Recorder::Recorder(std::string  pathToDirectory) : Module("/rosbag_record"), m_PathToDirectory{std::move(pathToDirectory)}
{

}

void Recorder::AfterRun()
{
    m_SpeedCalculatorFuture = std::async(std::launch::async, [this]{CalculateRecSpeed();});
}

void Recorder::CalculateRecSpeed()
{
    while(IsRunning())
    {
        m_FreeSpace = GetFreeSpace();

        if(!IsActive())
        {
            m_RecSpeed = 0;
            m_RecSize = 0;
            m_PrevSize = 0;
        }

        else
        {

        }

        using namespace std::chrono_literals;
        m_SpeedCalculatorFuture.wait_for(1s);
    }
}

double Recorder::GetFreeSpace()
{
    struct statfs info{};

    if(m_PathToDirectory.empty())
        return -1;

    while(statfs(m_PathToDirectory.data(), &info) == -1)
    {
        Log("Failed to get filesystem information", ERROR_LEVEL_LOG);
        Log("Trying again after some time", WARN_LEVEL_LOG);
    }

    return std::round((info.f_bavail * info.f_bsize) / 1000);
}

double Recorder::GetSize()
{
    if(m_CurrentDirectory.empty())
        return 0;

    struct stat file_stat{};

    while(stat(m_CurrentDirectory.data(), &file_stat) == -1)
    {
        Log("Error getting file size", ERROR_LEVEL_LOG);
        Log("Trying again after some time", WARN_LEVEL_LOG);
    }

    file_stat.st_size;

    for(const auto& path : std::filesystem::directory_iterator(m_CurrentDirectory.data()))
    {

    }
}
