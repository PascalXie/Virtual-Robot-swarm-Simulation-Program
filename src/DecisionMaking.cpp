#include "DecisionMaking.h"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
DecisionMaking::DecisionMaking()
{
    isTaskUnitSet = false;
    isMapSet = false;

}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
DecisionMaking::~DecisionMaking()
{}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
void DecisionMaking::setATaskUnit(AbstractTaskUnit* taskUnit_)
{
    taskUnit = taskUnit_;

    isTaskUnitSet = true;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......
// set the map
void DecisionMaking::setTheMap(std::vector<std::vector<int>> map_)
{
    map = map_;

    isMapSet = true;
}
