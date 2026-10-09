// Copyright (c) SIM A project.
#pragma once

#include "CoreMinimal.h"
#include "Modules/ModuleInterface.h"

/**
 * Runtime module for the Chrono bridge.
 *
 * A UE "module" is a shared library plus a registration record. IModuleInterface
 * is the contract: the engine calls StartupModule() after the .so is loaded and
 * ShutdownModule() before it is unloaded. Modules are the unit of both linkage
 * and lifetime in Unreal -- which is why the Chrono dependency belongs to a
 * module of its own rather than to game code.
 *
 * LoadingPhase in the .uplugin is "PreDefault": early enough that gameplay
 * modules can rely on the bridge existing, late enough that Core is up.
 *
 * NOTE: this header deliberately does NOT include chrono_shim.h. Public headers
 * are visible to every module that depends on us; keeping the shim private
 * means no engine translation unit ever sees even the C ABI.
 */
class CHRONOBRIDGE_API FChronoBridgeModule : public IModuleInterface
{
public:
    /** IModuleInterface */
    virtual void StartupModule() override;
    virtual void ShutdownModule() override;

    /** Convenience accessor; asserts if the module is not loaded. */
    static FChronoBridgeModule& Get()
    {
        return FModuleManager::LoadModuleChecked<FChronoBridgeModule>("ChronoBridge");
    }

    /** True once the shim reported a compatible ABI version. */
    bool IsChronoReady() const { return bChronoReady; }

    /** Chrono version string as reported by the shim, or empty. */
    const FString& GetChronoVersion() const { return ChronoVersion; }

private:
    bool    bChronoReady = false;
    FString ChronoVersion;
};
