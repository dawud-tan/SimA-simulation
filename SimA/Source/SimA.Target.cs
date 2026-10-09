using UnrealBuildTool;

// A .Target.cs describes an EXECUTABLE (game, editor, server), whereas a
// .Build.cs describes a MODULE (a shared library). The target names which
// modules get linked into the binary.
public class SimATarget : TargetRules
{
    public SimATarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Game;
        DefaultBuildSettings = BuildSettingsVersion.Latest;
        IncludeOrderVersion   = EngineIncludeOrderVersion.Latest;
        ExtraModuleNames.Add("SimA");
    }
}
