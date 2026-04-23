#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "PIDController.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "DronePawn.generated.h"

UCLASS()
class DRONE_SIM_API ADronePawn : public APawn
{
    GENERATED_BODY()

public:
    ADronePawn();

protected:
    virtual void BeginPlay() override;
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(
        class UInputComponent* PlayerInputComponent) override;

    // ── Skeletal Mesh ────────────────────────────────────────
    UPROPERTY(VisibleAnywhere, Category = "Drone")
        USkeletalMeshComponent* DroneMesh;

    // ── Rotor Static Meshes ──────────────────────────────────
    UPROPERTY(VisibleAnywhere, Category = "Drone")
        UStaticMeshComponent* Rotor_FL;

    UPROPERTY(VisibleAnywhere, Category = "Drone")
        UStaticMeshComponent* Rotor_FR;

    UPROPERTY(VisibleAnywhere, Category = "Drone")
        UStaticMeshComponent* Rotor_BL;

    UPROPERTY(VisibleAnywhere, Category = "Drone")
        UStaticMeshComponent* Rotor_BR;

    // ── Cameras ──────────────────────────────────────────────
    UPROPERTY(VisibleAnywhere, Category = "Drone")
        class USpringArmComponent* SpringArm;

    UPROPERTY(VisibleAnywhere, Category = "Drone")
        class UCameraComponent* Camera;

    UPROPERTY(VisibleAnywhere, Category = "Drone")
        class UCameraComponent* FPVCamera;

    // ── Physics ──────────────────────────────────────────────
    UPROPERTY(EditAnywhere, Category = "Drone|Physics")
        float Mass = 1.5f;

    UPROPERTY(EditAnywhere, Category = "Drone|Physics")
        float MaxRotorThrust = 600.f;

    UPROPERTY(EditAnywhere, Category = "Drone|Physics")
        float MaxTiltAngle = 35.f;

    // ── PID Altitude ─────────────────────────────────────────
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Altitude")
        float Alt_Kp = 8.f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Altitude")
        float Alt_Ki = 0.1f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Altitude")
        float Alt_Kd = 4.f;

    // ── PID Pitch ────────────────────────────────────────────
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Pitch")
        float Pitch_Kp = 6.f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Pitch")
        float Pitch_Ki = 0.05f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Pitch")
        float Pitch_Kd = 3.f;

    // ── PID Roll ─────────────────────────────────────────────
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Roll")
        float Roll_Kp = 6.f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Roll")
        float Roll_Ki = 0.05f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Roll")
        float Roll_Kd = 3.f;

    // ── PID Yaw ──────────────────────────────────────────────
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Yaw")
        float Yaw_Kp = 4.f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Yaw")
        float Yaw_Ki = 0.01f;
    UPROPERTY(EditAnywhere, Category = "Drone|PID|Yaw")
        float Yaw_Kd = 2.f;

    // ── Rotor Settings ───────────────────────────────────────
    UPROPERTY(EditAnywhere, Category = "Drone|Rotors")
        float MaxRotorSpeed = 3000.f;

public:
    // ── HUD Getters ──────────────────────────────────────────
    UFUNCTION(BlueprintCallable) float GetSpeed()    const;
    UFUNCTION(BlueprintCallable) float GetAltitude() const;
    UFUNCTION(BlueprintCallable) float GetRotorFL()  const;
    UFUNCTION(BlueprintCallable) float GetRotorFR()  const;
    UFUNCTION(BlueprintCallable) float GetRotorBL()  const;
    UFUNCTION(BlueprintCallable) float GetRotorBR()  const;
    UFUNCTION(BlueprintCallable) bool  IsCrashed()   const;

private:
    // ── PIDs ─────────────────────────────────────────────────
    FPIDController PID_Altitude;
    FPIDController PID_Pitch;
    FPIDController PID_Roll;
    FPIDController PID_Yaw;

    // ── Input ────────────────────────────────────────────────
    float InputThrottle = 0.f;
    float InputPitch = 0.f;
    float InputRoll = 0.f;
    float InputYaw = 0.f;

    // ── Physics State ────────────────────────────────────────
    FVector Velocity = FVector::ZeroVector;
    FVector PreviousVelocity = FVector::ZeroVector;

    // ── Per Rotor Thrust ─────────────────────────────────────
    float RotorThrust_FL = 0.f;
    float RotorThrust_FR = 0.f;
    float RotorThrust_BL = 0.f;
    float RotorThrust_BR = 0.f;

    // ── Rotor Angles ─────────────────────────────────────────
    float RotorAngle_FL = 0.f;
    float RotorAngle_FR = 0.f;
    float RotorAngle_BL = 0.f;
    float RotorAngle_BR = 0.f;

    // ── Targets ──────────────────────────────────────────────
    float TargetAltitude = 0.f;
    float TargetYaw = 0.f;

    // ── State ────────────────────────────────────────────────
    bool bCrashed = false;
    bool bIsFPV = false;

    // ── Functions ────────────────────────────────────────────
    void CalculateRotorSpeeds(float DeltaTime);
    void ApplyRotorPhysics(float DeltaTime);
    void SpinRotors(float DeltaTime);
    void CheckCrash();
    void ResetDrone();
    void ToggleFPV();

    // ── Input Callbacks ──────────────────────────────────────
    void OnThrottle(float Val);
    void OnPitch(float Val);
    void OnRoll(float Val);
    void OnYaw(float Val);
};