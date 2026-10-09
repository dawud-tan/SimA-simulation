#include "Modules/ModuleManager.h"

// Primary game module. IMPLEMENT_PRIMARY_GAME_MODULE differs from
// IMPLEMENT_MODULE in that it also defines the engine's entry point glue.
IMPLEMENT_PRIMARY_GAME_MODULE(FDefaultGameModuleImpl, SimA, "SimA");
