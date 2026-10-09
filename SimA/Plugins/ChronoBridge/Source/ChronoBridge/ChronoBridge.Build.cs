// Copyright (c) SIM A project.
//
// A .Build.cs is not a config file -- it is a C# CLASS that UnrealBuildTool
// compiles and instantiates at build time. UBT then reads its fields to decide
// include paths, link libraries, and which other modules this one may see.
// Because it is real code, it can branch on target platform, configuration,
// and the filesystem, which is exactly what we need for a third-party .so.

using UnrealBuildTool;
using System.IO;

public class ChronoBridge : ModuleRules
{
    public ChronoBridge(ReadOnlyTargetRules Target) : base(Target)
    {
        // Unreal's precompiled-header strategy. "UseExplicitOrSharedPCHs" is
        // the modern default and keeps our translation units from silently
        // inheriting engine headers we did not ask for.
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        // ---- Engine dependencies -------------------------------------
        // PublicDependencyModuleNames  : modules our PUBLIC headers expose.
        // PrivateDependencyModuleNames : modules only our .cpp files use.
        // Keeping Chrono out of BOTH is the point of this plugin: no engine
        // module ever gains visibility of a Chrono type.
        // These are PUBLIC because our public headers use them:
        // ChronoWorldSubsystem.h derives from UTickableWorldSubsystem (Engine)
        // and uses UCLASS/GENERATED_BODY (CoreUObject). A module whose public
        // header exposes a type must publicly depend on that type's module, or
        // dependents fail to compile with missing includes.
        PublicDependencyModuleNames.AddRange(new string[] {
            "Core",
            "CoreUObject",
            "Engine",
        });

        PrivateDependencyModuleNames.AddRange(new string[] {
            "Projects",   // IPluginManager, for locating our own plugin dir
            // EKeys, for the Milestone 6 keyboard fallback. Engine already
            // pulls InputCore in transitively, but a module that names a
            // dependency it actually uses does not break when the engine
            // reshuffles its own public dependency list.
            "InputCore",
            // FJsonSerializer: the vehicle actor reads the driver's seat and
            // the steering range from the vehicle's own JSON (task 9).
            "Json",
            // The frame's own timers for the health line -- the numbers
            // `stat unit` shows: GGameThreadTime and friends (RenderCore),
            // RHIGetGPUFrameCycles (RHI). Task 18.
            "RenderCore",
            "RHI",
        });

        // ---- The Chrono shim ------------------------------------------
        // We link ONLY libchrono_shim.so, never libChrono_core.so directly.
        // The shim exports 7 extern "C" symbols; Chrono's 28574 C++ symbols
        // are made local by a linker version script and are invisible here.
        if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // Locate chrono-sdk by walking up from the plugin directory until
            // we find it. Hard-coding a "../../.." chain is brittle -- one
            // wrong level and the error only surfaces at generate time.
            string SdkDir = null;
            for (DirectoryInfo Dir = new DirectoryInfo(PluginDirectory);
                 Dir != null; Dir = Dir.Parent)
            {
                string Candidate = Path.Combine(Dir.FullName, "chrono-sdk");
                if (Directory.Exists(Candidate)) { SdkDir = Candidate; break; }
            }
            if (SdkDir == null)
            {
                throw new BuildException(
                    "chrono-sdk not found in any parent of {0}. Run docker/03-export-sdk.sh.",
                    PluginDirectory);
            }
            string LibDir = Path.Combine(SdkDir, "lib");
            string ShimLib = Path.Combine(LibDir, "libchrono_shim.so");

            if (!File.Exists(ShimLib))
            {
                throw new BuildException(
                    "libchrono_shim.so not found at {0}. Run docker/04-build-shim.sh first.",
                    ShimLib);
            }

            // PublicIncludePaths: where chrono_shim.h lives. It must be PUBLIC
            // because ChronoStateRing.h is a public header that includes it.
            //
            // That is safe, and is precisely the point of the wall: chrono_shim.h
            // is pure C with POD structs only. Exposing it to dependent modules
            // cannot contaminate them, because there is nothing in it whose
            // layout depends on -march or on which C++ runtime is in use. What
            // must never become reachable is a CHRONO header -- and none is.
            PublicIncludePaths.Add(Path.Combine(SdkDir, "include"));

            // PublicAdditionalLibraries: what the linker resolves against.
            PublicAdditionalLibraries.Add(ShimLib);

            // RuntimeDependencies: files that must exist next to the packaged
            // binary. Without this the editor links fine but a packaged build
            // dies at startup with an unresolved .so.
            RuntimeDependencies.Add("$(BinaryOutputDir)/libchrono_shim.so", ShimLib);

            // The dynamic loader needs to FIND the .so at runtime. UE sets
            // $ORIGIN-relative rpath for its own binaries; we add the SDK dir
            // so an in-editor run resolves without LD_LIBRARY_PATH.
            PublicRuntimeLibraryPaths.Add(LibDir);

            // Chrono itself is a transitive dependency of the shim: the loader
            // must find libChrono_core.so when it loads libchrono_shim.so.
            foreach (string Dll in Directory.GetFiles(LibDir, "libChrono*.so*"))
            {
                RuntimeDependencies.Add(
                    "$(BinaryOutputDir)/" + Path.GetFileName(Dll), Dll);
            }
        }

        // Chrono is built with exceptions enabled and the shim catches all of
        // them internally, so none can unwind into UE. UE code itself stays
        // exception-free, which is the engine default.
        bEnableExceptions = false;
    }
}
