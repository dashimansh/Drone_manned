#pragma once
#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "DroneHUD.generated.h"

UCLASS()
class DRONE_SIM_API ADroneHUD : public AHUD
{
    GENERATED_BODY()
public:
    virtual void DrawHUD() override;
private:
    void DrawText(
        const FString& Text,
        float X, float Y,
        FLinearColor Color);
};