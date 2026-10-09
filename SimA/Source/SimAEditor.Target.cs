using UnrealBuildTool;

public class SimAEditorTarget : TargetRules
{
    public SimAEditorTarget(TargetInfo Target) : base(Target)
    {
        Type = TargetType.Editor;
        DefaultBuildSettings = BuildSettingsVersion.Latest;
        IncludeOrderVersion   = EngineIncludeOrderVersion.Latest;
        ExtraModuleNames.Add("SimA");
    }
}
