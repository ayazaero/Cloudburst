using UnrealBuildTool;
using System.IO;

public class UE : ModuleRules
{
    public UE(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "Sockets", "Networking","Json", "JsonUtilities"});

        PublicIncludePaths.AddRange(
            new string[] {
                Path.Combine(ModuleDirectory, "Public"),
                Path.Combine(ModuleDirectory, "../ThirdParty/ZeroMQ/include")
            }
        );

        PrivateDependencyModuleNames.AddRange(new string[] { "ZeroMQPlugin" });

        PrivateIncludePathModuleNames.AddRange(new string[] { "ZeroMQPlugin" });

        // Uncomment if you are using Slate UI
        // PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

        // Uncomment if you are using online features
        // PrivateDependencyModuleNames.Add("OnlineSubsystem");

        // To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true

        RuntimeDependencies.Add(Path.Combine(ModuleDirectory, "../Binaries/Win64/libzmq-mt-gd-4_3_6.dll"));

    }
}