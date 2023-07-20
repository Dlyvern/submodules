#include "Recorder.h"

#include <utility>

Recorder::Recorder(std::string  pathToDirectory, std::shared_ptr<Robot>&parent) : Module("/rosbag_record"), m_PathToDirectory{std::move(pathToDirectory)}, m_Robot{parent}{}

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
            uint32_t size = GetSize();

            if(m_Speeds.size() < 10)
                m_Speeds.emplace_back(std::max((size - m_PrevSize), (uint32_t)0));

            uint32_t sum_of_elements{0};

            std::for_each(m_Speeds.begin(), m_Speeds.end(), [&](uint32_t element){sum_of_elements += element;});

            m_RecSpeed = std::round(sum_of_elements / m_Speeds.size());
            m_RecSize = size;
            m_PrevSize = size;
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

uint32_t Recorder::GetSize()
{
    if(m_CurrentDirectory.empty())
        return 0;

    uint32_t total_size{0};

    //We iterate over each entry in the directory that we check if it is a regular file if it is, we add its size to the total_size variable
    for(const auto& entry : std::filesystem::directory_iterator(m_CurrentDirectory))
        if(entry.is_regular_file())
            total_size += std::filesystem::file_size(entry);

    return total_size / 1024; //Getting the result in kb
}

std::string Recorder::Pause(RunParameters &runParameters)
{
    m_Pause = true;
    return{};
}

std::string Recorder::UnPause(RunParameters &runParameters)
{
    m_Pause = false;
    return{};
}

std::string Recorder::Check(bool silent, bool kill)
{
    std::vector<std::string> node_names = this->get_node_names();

    auto run_parameters = RunParameters();

    //If we could not find our node name in node_names vector
    if(!std::any_of(node_names.begin(), node_names.end(), [this](const std::string &node_name) { return node_name == m_NodeName; }))
    {
        Disable(run_parameters);
        if(!silent)
            Log("Recorder failed", ERROR_LEVEL_LOG);
        return{"False"};
    }

    m_FragmentNum = ((std::chrono::system_clock::now() - m_TimeStart).count() / m_Duration);

    return"True";
}

std::string Recorder::Disable(RunParameters &runParameters)
{
    if(!IsActive())
    {
        Log("disabled already!", WARN_LEVEL_LOG);
        return "already_disabled";
    }

    SetActive(false);

    m_Speeds.clear();

    m_PrevSize = 0;

    try
    {
        Log("Trying to kill the " + m_NodeName, INFO_LEVEL_LOG);
        rclcpp::shutdown();
        Log(m_NodeName + " was killed successfully", INFO_LEVEL_LOG);
    }

    catch(std::exception&exception)
    {
        Log("Could not kill the " + m_NodeName + " exception: " + exception.what(), WARN_LEVEL_LOG);
    }

    Log("Recording stopped", INFO_LEVEL_LOG);

    std::string gps_in_string = m_Robot->GetGps();

    std::vector<double> gps;

    //For example, gps_in_string = "[10394.72938, 17639.8378]"
    //In gps we have to put 10394.72938 and 17639.8378

    //Position from the last comma
    size_t start_position{0};

    //Get index of [
    uint32_t bracket_open_index = gps_in_string.find('[');

    //Delete it
    gps_in_string.erase(gps_in_string.begin() + bracket_open_index);

    //Get index of ]
    uint32_t bracket_closed_index = gps_in_string.find(']');

    //Delete it
    gps_in_string.erase(gps_in_string.begin() + bracket_closed_index);

    // //Finds comma
    size_t comma_index = gps_in_string.find(',');

    //While we can find the last comma
    while(comma_index != std::string::npos)
    {
        gps.emplace_back(std::stod(gps_in_string.substr(start_position, comma_index - start_position)));

        //Getting position after the comma +2 because here (8, 1) the distance between comma and next number is 2
        start_position = comma_index + 2;

        //Finds new comma from new position
        comma_index = gps_in_string.find(',', start_position);
    }

    gps.emplace_back(std::stod(gps_in_string.substr(start_position)));

}
