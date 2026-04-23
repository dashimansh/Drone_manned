#include "DroneHUD.h"
#include "DronePawn.h"
#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "GameFramework/PlayerController.h"
#include "CanvasItem.h"

void ADroneHUD::DrawHUD()
{
    Super::DrawHUD();

    APlayerController* PC = GetOwningPlayerController();
    if (!PC) return;

    ADronePawn* Drone = Cast<ADronePawn>(PC->GetPawn());
    if (!Drone) return;

    float Speed = Drone->GetSpeed();
    float Altitude = Drone->GetAltitude();
    float FL = Drone->GetRotorFL();
    float FR = Drone->GetRotorFR();
    float BL = Drone->GetRotorBL();
    float BR = Drone->GetRotorBR();
    bool  Crashed = Drone->IsCrashed();

    FLinearColor White = FLinearColor::White;
    FLinearColor Red = FLinearColor::Red;
    FLinearColor Green = FLinearColor::Green;
    FLinearColor Cyan = FLinearColor(0.f, 1.f, 1.f, 1.f);
    FLinearColor Gray = FLinearColor(0.5f, 0.5f, 0.5f, 1.f);
    FLinearColor Yellow = FLinearColor::Yellow;

    // ── Title ────────────────────────────────────────────────
    DrawText(TEXT("ROTOR PHYSICS DRONE"), 20, 20, Cyan);

    // ── Flight Data ──────────────────────────────────────────
    DrawText(FString::Printf(TEXT("Speed:    %.0f cm/s"), Speed),
        20, 50, White);
    DrawText(FString::Printf(TEXT("Altitude: %.0f cm"), Altitude),
        20, 75, White);

    // ── Rotor Thrust ─────────────────────────────────────────
    DrawText(TEXT("── Rotor Thrust ──"), 20, 110, Yellow);
    DrawText(FString::Printf(TEXT("FL: %.0f"), FL), 20, 135, Green);
    DrawText(FString::Printf(TEXT("FR: %.0f"), FR), 150, 135, Green);
    DrawText(FString::Printf(TEXT("BL: %.0f"), BL), 20, 160, Green);
    DrawText(FString::Printf(TEXT("BR: %.0f"), BR), 150, 160, Green);

    // ── Diagram ──────────────────────────────────────────────
    DrawText(TEXT("FL ↺  ↻ FR"), 20, 195, Gray);
    DrawText(TEXT("  [DRONE]"), 20, 215, Gray);
    DrawText(TEXT("BL ↻  ↺ BR"), 20, 235, Gray);

    // ── Controls ─────────────────────────────────────────────
    DrawText(TEXT("W/S=Throttle | Arrows=Pitch/Roll"), 20, 270, Gray);
    DrawText(TEXT("Q/E=Yaw | C=FPV | T=Reset"), 20, 290, Gray);

    // ── Crash ────────────────────────────────────────────────
    if (Crashed && Canvas)
    {
        DrawText(TEXT("DRONE CRASHED! Press T!"),
            Canvas->SizeX / 2.f - 180.f,
            Canvas->SizeY / 2.f, Red);
    }
}

void ADroneHUD::DrawText(
    const FString& Text, float X, float Y, FLinearColor Color)
{
    if (!Canvas || !GEngine) return;

    FCanvasTextItem TextItem(
        FVector2D(X, Y),
        FText::FromString(Text),
        GEngine->GetMediumFont(),
        Color);

    TextItem.Scale = FVector2D(1.5f, 1.5f);
    Canvas->DrawItem(TextItem);
}