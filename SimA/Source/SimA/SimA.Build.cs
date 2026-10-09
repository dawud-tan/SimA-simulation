using UnrealBuildTool;

public class SimA : ModuleRules
{
    public SimA(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] {
            "Core", "CoreUObject", "Engine", "InputCore",
            // Depending on the bridge MODULE gives us its public header only.
            // Chrono stays invisible: the shim include path is private to the
            // ChronoBridge module, so nothing here can accidentally include it.
            "ChronoBridge",
        });
    }
}
