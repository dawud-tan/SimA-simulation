// Copyright (c) SIM A project.

#include "ChronoBridgeModule.h"

#include "Modules/ModuleManager.h"

// The ONE place the C ABI is included. It is a pure C header: no Chrono types,
// no Eigen, no std::. Including it here cannot change the layout of anything
// in a UE translation unit, which is the entire reason it exists.
//
// THIS_IS_LOAD_BEARING: never replace this with a Chrono header. See the
// comment block at the top of chrono_shim.h for the measured consequences.
#include "chrono_shim.h"

#define LOCTEXT_NAMESPACE "FChronoBridgeModule"

DEFINE_LOG_CATEGORY_STATIC(LogChronoBridge, Log, All);

void FChronoBridgeModule::StartupModule()
{
    // Reaching this point already proves the dynamic loader resolved
    // libchrono_shim.so and, transitively, libChrono_core.so -- if it had not,
    // the module .so itself would have failed to load and we would never run.
    const int32 ShimAbi = Chs_GetAbiVersion();

    if (ShimAbi != CHRONO_SHIM_ABI_VERSION)
    {
        // Header and .so disagree. This is the failure mode that would
        // otherwise be silent memory corruption, so we make it loud and refuse
        // to mark ourselves ready.
        UE_LOG(LogChronoBridge, Error,
               TEXT("ABI mismatch: libchrono_shim.so reports %d, header expects %d. ")
               TEXT("Rebuild the shim (docker/04-build-shim.sh)."),
               ShimAbi, CHRONO_SHIM_ABI_VERSION);
        bChronoReady = false;
        return;
    }

    // Fixed caller-owned buffer. The shim copies into it rather than returning
    // a pointer, so no allocation crosses the heap boundary.
    char VersionBuf[64] = {};
    Chs_GetChronoVersion(VersionBuf, sizeof(VersionBuf));
    ChronoVersion = FString(ANSI_TO_TCHAR(VersionBuf));

    bChronoReady = true;

    UE_LOG(LogChronoBridge, Log,
           TEXT("Chrono bridge ready. Chrono %s, shim ABI %d."),
           *ChronoVersion, ShimAbi);
}

void FChronoBridgeModule::ShutdownModule()
{
    // Nothing global is allocated at startup, so there is nothing to release.
    // Worlds created via Chs_CreateWorld are owned by whoever created them and
    // must be destroyed with Chs_DestroyWorld -- crucially, by the shim, since
    // it allocated them on ITS heap, not Unreal's.
    bChronoReady = false;
    UE_LOG(LogChronoBridge, Log, TEXT("Chrono bridge shut down."));
}

#undef LOCTEXT_NAMESPACE

// Registers the module with UE's module manager. The macro emits the exported
// factory function that UnrealBuildTool wires into the module registry, and
// the string must match the module name in ChronoBridge.uplugin exactly.
IMPLEMENT_MODULE(FChronoBridgeModule, ChronoBridge)
