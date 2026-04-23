#include "DronePawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Math/UnrealMathUtility.h"

ADronePawn::ADronePawn()
{
    PrimaryActorTick.bCanEverTick = true;

    // ── Skeletal Mesh ────────────────────────────────────────
    DroneMesh = CreateDefaultSubobject<USkeletalMeshComponent>(
        TEXT("DroneMesh"));
    RootComponent = DroneMesh;
    DroneMesh->SetSimulatePhysics(false);
    DroneMesh->SetRelativeRotation(FRotator(0, -90, 0));

    // ── Rotors attached to skeleton bones ────────────────────
    Rotor_FL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor_FL"));
    Rotor_FL->SetupAttachment(DroneMesh, TEXT("ROTOR_12"));
    Rotor_FL->SetRelativeLocation(FVector(0.f, 0.f, 2.f));
    Rotor_FL->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
    Rotor_FL->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));

    Rotor_FR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor_FR"));
    Rotor_FR->SetupAttachment(DroneMesh, TEXT("ROTOR_21"));
    Rotor_FR->SetRelativeLocation(FVector(0.f, 0.f, 2.f));
    Rotor_FR->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
    Rotor_FR->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));

    Rotor_BL = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor_BL"));
    Rotor_BL->SetupAttachment(DroneMesh, TEXT("ROTOR_33"));
    Rotor_BL->SetRelativeLocation(FVector(0.f, 0.f, 2.f));
    Rotor_BL->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
    Rotor_BL->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));

    Rotor_BR = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Rotor_BR"));
    Rotor_BR->SetupAttachment(DroneMesh, TEXT("ROTOR_44"));
    Rotor_BR->SetRelativeLocation(FVector(0.f, 0.f, 2.f));
    Rotor_BR->SetRelativeRotation(FRotator(90.f, 0.f, 0.f));
    Rotor_BR->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));

    // ── Spring Arm ───────────────────────────────────────────
    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(DroneMesh);
    SpringArm->TargetArmLength = 200.f;
    SpringArm->SetRelativeLocation(FVector(0, 0, 0));
    SpringArm->SetRelativeRotation(FRotator(-20, 0, 0));
    SpringArm->bUsePawnControlRotation = false;
    SpringArm->bInheritYaw = true;
    SpringArm->bInheritPitch = false;
    SpringArm->bInheritRoll = false;

    // ── Third Person Camera ──────────────────────────────────
    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm, USpringArmComponent::SocketName);
    Camera->SetActive(true);

    // ── FPV Camera ───────────────────────────────────────────
    FPVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("FPVCamera"));
    FPVCamera->SetupAttachment(DroneMesh);
    FPVCamera->SetRelativeLocation(FVector(15, 0, 5));
    FPVCamera->SetRelativeRotation(FRotator(0, 90, 0));
    FPVCamera->SetActive(false);
}

void ADronePawn::BeginPlay()
{
    Super::BeginPlay();

    // ── Force correct rotor setup at runtime ─────────────────
    auto SetupRotor = [](UStaticMeshComponent* R)
    {
        if (!R) return;
        R->SetRelativeScale3D(FVector(0.5f, 0.5f, 0.05f));
        R->SetRelativeLocation(FVector(0.f, 0.f, 2.f));
    };

    SetupRotor(Rotor_FL);
    SetupRotor(Rotor_FR);
    SetupRotor(Rotor_BL);
    SetupRotor(Rotor_BR);

    // ── Initialize PIDs ──────────────────────────────────────
    PID_Altitude = FPIDController(Alt_Kp, Alt_Ki, Alt_Kd);
    PID_Pitch = FPIDController(Pitch_Kp, Pitch_Ki, Pitch_Kd);
    PID_Roll = FPIDController(Roll_Kp, Roll_Ki, Roll_Kd);
    PID_Yaw = FPIDController(Yaw_Kp, Yaw_Ki, Yaw_Kd);

    TargetAltitude = GetActorLocation().Z;
    TargetYaw = GetActorRotation().Yaw;
}

void ADronePawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (!bCrashed)
    {
        CalculateRotorSpeeds(DeltaTime);
        ApplyRotorPhysics(DeltaTime);
        SpinRotors(DeltaTime);
        CheckCrash();
    }
}

void ADronePawn::CalculateRotorSpeeds(float DeltaTime)
{
    float CurrentAlt = GetActorLocation().Z;
    float CurrentPitch = GetActorRotation().Pitch;
    float CurrentRoll = GetActorRotation().Roll;
    float CurrentYawR = GetActorRotation().Yaw;

    TargetAltitude += InputThrottle * 200.f * DeltaTime;
    TargetYaw += InputYaw * 90.f * DeltaTime;

    float AltOut = PID_Altitude.Update(
        TargetAltitude - CurrentAlt, DeltaTime);
    float PitchOut = PID_Pitch.Update(
        -InputPitch * MaxTiltAngle - CurrentPitch, DeltaTime);
    float RollOut = PID_Roll.Update(
        InputRoll * MaxTiltAngle - CurrentRoll, DeltaTime);
    float YawOut = PID_Yaw.Update(
        TargetYaw - CurrentYawR, DeltaTime);

    float Base = FMath::Clamp(AltOut, 0.f, MaxRotorThrust);
    PitchOut = FMath::Clamp(PitchOut, -200.f, 200.f);
    RollOut = FMath::Clamp(RollOut, -200.f, 200.f);
    YawOut = FMath::Clamp(YawOut, -100.f, 100.f);

    RotorThrust_FL = FMath::Clamp(
        Base + PitchOut + RollOut - YawOut, 0.f, MaxRotorThrust);
    RotorThrust_FR = FMath::Clamp(
        Base + PitchOut - RollOut + YawOut, 0.f, MaxRotorThrust);
    RotorThrust_BL = FMath::Clamp(
        Base - PitchOut + RollOut + YawOut, 0.f, MaxRotorThrust);
    RotorThrust_BR = FMath::Clamp(
        Base - PitchOut - RollOut - YawOut, 0.f, MaxRotorThrust);
}

void ADronePawn::ApplyRotorPhysics(float DeltaTime)
{
    const float Gravity = -980.f;

    float TotalThrust = RotorThrust_FL + RotorThrust_FR
        + RotorThrust_BL + RotorThrust_BR;

    float PitchTorque = (RotorThrust_FL + RotorThrust_FR)
        - (RotorThrust_BL + RotorThrust_BR);
    float RollTorque = (RotorThrust_FL + RotorThrust_BL)
        - (RotorThrust_FR + RotorThrust_BR);
    float YawTorque = (RotorThrust_FR + RotorThrust_BL)
        - (RotorThrust_FL + RotorThrust_BR);

    FRotator CurrentRot = GetActorRotation();
    float NewPitch = FMath::Clamp(
        CurrentRot.Pitch + PitchTorque * 0.0005f, -45.f, 45.f);
    float NewRoll = FMath::Clamp(
        CurrentRot.Roll + RollTorque * 0.0005f, -45.f, 45.f);
    float NewYaw = CurrentRot.Yaw + YawTorque * 0.0002f;

    SetActorRotation(FRotator(NewPitch, NewYaw, NewRoll));

    FVector Up = GetActorUpVector();
    FVector Accel = Up * (TotalThrust / Mass)
        + FVector(0, 0, Gravity);

    PreviousVelocity = Velocity;
    Velocity += Accel * DeltaTime;
    Velocity *= FMath::Clamp(1.f - 0.5f * DeltaTime, 0.f, 1.f);

    FVector NewPos = GetActorLocation() + Velocity * DeltaTime;
    if (NewPos.Z < 0.f) { NewPos.Z = 0.f; Velocity.Z = 0.f; }
    SetActorLocation(NewPos);
}

void ADronePawn::SpinRotors(float DeltaTime)
{
    auto SpinRotor = [&](UStaticMeshComponent* Rotor,
        float& Angle,
        float  Thrust,
        bool   bCW)
    {
        if (!Rotor) return;
        float SpinRate = (Thrust / MaxRotorThrust) * MaxRotorSpeed;
        Angle = FMath::Fmod(
            Angle + (bCW ? SpinRate : -SpinRate) * DeltaTime,
            360.f);
        // 90 degree base keeps disc flat
        // Yaw angle spins horizontally
        Rotor->SetRelativeRotation(FRotator(90.f, Angle, 0.f));
    };

    SpinRotor(Rotor_FL, RotorAngle_FL, RotorThrust_FL, false);
    SpinRotor(Rotor_FR, RotorAngle_FR, RotorThrust_FR, true);
    SpinRotor(Rotor_BL, RotorAngle_BL, RotorThrust_BL, true);
    SpinRotor(Rotor_BR, RotorAngle_BR, RotorThrust_BR, false);
}

void ADronePawn::CheckCrash()
{
    float ImpactForce = (Velocity - PreviousVelocity).Size();
    if (ImpactForce > 1000.f && GetActorLocation().Z < 5.f)
    {
        bCrashed = true;
        Velocity = FVector::ZeroVector;
        PID_Altitude.Reset();
        PID_Pitch.Reset();
        PID_Roll.Reset();
        PID_Yaw.Reset();
    }
}

void ADronePawn::ResetDrone()
{
    bCrashed = false;
    Velocity = FVector::ZeroVector;
    RotorThrust_FL = RotorThrust_FR = 0.f;
    RotorThrust_BL = RotorThrust_BR = 0.f;
    TargetAltitude = 100.f;
    TargetYaw = 0.f;

    SetActorLocation(FVector(0, 0, 100));
    SetActorRotation(FRotator::ZeroRotator);

    PID_Altitude.Reset();
    PID_Pitch.Reset();
    PID_Roll.Reset();
    PID_Yaw.Reset();
}

void ADronePawn::ToggleFPV()
{
    bIsFPV = !bIsFPV;
    FPVCamera->SetActive(bIsFPV);
    Camera->SetActive(!bIsFPV);
}

void ADronePawn::SetupPlayerInputComponent(
    UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    PlayerInputComponent->BindAxis(
        "Throttle", this, &ADronePawn::OnThrottle);
    PlayerInputComponent->BindAxis(
        "Pitch", this, &ADronePawn::OnPitch);
    PlayerInputComponent->BindAxis(
        "Roll", this, &ADronePawn::OnRoll);
    PlayerInputComponent->BindAxis(
        "Yaw", this, &ADronePawn::OnYaw);

    PlayerInputComponent->BindAction(
        "ResetDrone", IE_Pressed, this, &ADronePawn::ResetDrone);
    PlayerInputComponent->BindAction(
        "ToggleFPV", IE_Pressed, this, &ADronePawn::ToggleFPV);
}

void ADronePawn::OnThrottle(float Val) { InputThrottle = Val; }
void ADronePawn::OnPitch(float Val) { InputPitch = Val; }
void ADronePawn::OnRoll(float Val) { InputRoll = Val; }
void ADronePawn::OnYaw(float Val) { InputYaw = Val; }

float ADronePawn::GetSpeed()    const { return Velocity.Size(); }
float ADronePawn::GetAltitude() const { return GetActorLocation().Z; }
float ADronePawn::GetRotorFL()  const { return RotorThrust_FL; }
float ADronePawn::GetRotorFR()  const { return RotorThrust_FR; }
float ADronePawn::GetRotorBL()  const { return RotorThrust_BL; }
float ADronePawn::GetRotorBR()  const { return RotorThrust_BR; }
bool  ADronePawn::IsCrashed()   const { return bCrashed; }