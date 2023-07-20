#ifndef RECORDER_H
#define RECORDER_H

#ifndef MODULE_H
#include "Module.h"
#endif

#ifndef RUN_PARAMETERS_H
#include "RunParameters.h"
#endif


#include "sys/statfs.h"
#include "sys/stat.h"
#include "filesystem"

class Recorder : public Module
{
private:
    std::string m_LaunchFileName{"./submodules/launch/rec.launch"};
    std::future<void> m_SpeedCalculatorFuture;
    std::string m_PathToDirectory{" "};
    std::string m_CurrentDirectory{" "};
    double m_FreeSpace{0};
    double m_RecSpeed{0};
    double m_RecSize{0};
    double m_PrevSize{0};
public:
    explicit Recorder(std::string  pathToDirectory);

    void AfterRun() override;

    void CalculateRecSpeed();


    double GetSize();


    double GetFreeSpace();

};

#endif //RECORDER_H
