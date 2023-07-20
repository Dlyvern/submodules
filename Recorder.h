#ifndef RECORDER_H
#define RECORDER_H

#ifndef MODULE_H
#include "Module.h"
#endif

#ifndef RUN_PARAMETERS_H
#include "RunParameters.h"
#endif

#ifndef ROBOT_H
#include "Robot.h"
#endif

#include "sys/statfs.h"
#include "sys/stat.h"
#include "filesystem"
#include "algorithm"

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/storage_options.hpp>

class Recorder : public Module
{
private:
    std::string m_LaunchFileName{"./submodules/launch/rec.launch"};
    std::future<void> m_SpeedCalculatorFuture;
    std::string m_PathToDirectory{" "};
    std::filesystem::path  m_CurrentDirectory{" "};
    std::string m_NodeName{"/rosbag_record"};

    std::shared_ptr<Robot>m_Robot;

    std::vector<double>m_StartPosition;

    double m_FreeSpace{0};
    double m_RecSpeed{0};
    double m_RecSize{0};
    uint32_t m_PrevSize{0};
    std::vector<uint32_t>m_Speeds;
    std::chrono::time_point<std::chrono::system_clock> m_TimeStart;
    uint32_t m_FragmentNum{0};
    uint16_t m_Duration{30};
    bool m_Pause{false};
public:
    explicit Recorder(std::string  pathToDirectory, std::shared_ptr<Robot>&parent);

    std::map<std::string, uint64_t> GetBagTopics(const std::string &fileName);

    std::map<std::string, uint64_t> GetRecordedTopics(const std::string &directoryName);

    std::string Check(bool silent = false, bool kill = false);

    void LogTopics();

    std::string Disable(RunParameters& runParameters) override;

    std::string Enable(RunParameters& runParameters) override;

    std::string Pause(RunParameters& runParameters);

    std::string UnPause(RunParameters& runParameters);

    void AfterRun() override;

    void CalculateRecSpeed();


    uint32_t GetSize();


    double GetFreeSpace();

};

#endif //RECORDER_H
