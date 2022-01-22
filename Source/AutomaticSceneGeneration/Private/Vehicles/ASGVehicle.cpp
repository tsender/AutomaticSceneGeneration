// Fill out your copyright notice in the Description page of Project Settings.


#include "Vehicles/ASGVehicle.h"
#include "Components/PIDDriveByWireComponent.h"
#include "Components/AnnotationComponent.h"
// #include "Components/VehicleEvaluationComponent.h"
#include "Components/AudioComponent.h"
#include "WheeledVehicleMovementComponent.h"
#include "Actors/StructuralSceneActor.h"
// #include "Actors/ASGWorker.h"
#include "Sound/SoundCue.h"
#include "Engine/TriggerVolume.h"

#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/std_msgs/Bool.h"
#include "ROSIntegration/Public/nav_msgs/Path.h"

AASGVehicle::AASGVehicle() 
{
    DriveByWireComponent = CreateDefaultSubobject<UPIDDriveByWireComponent>(TEXT("PID Drive By Wire"));
    AnnotationComponent = CreateDefaultSubobject<UAnnotationComponent>(TEXT("Annotation Component"));
    // EvaluationComponent = CreateDefaultSubobject<UVehicleEvaluationComponent>(TEXT("Vehicle Evaluation Component"));

    static ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/Game/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
	EngineSoundComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("EngineSound"));
	EngineSoundComponent->SetSound(SoundCue.Object);
	EngineSoundComponent->SetupAttachment(GetMesh());
}

void AASGVehicle::BeginPlay() 
{
    Super::BeginPlay();

    // Make sure skeletal mesh is set to generate hit events
    GetMesh()->OnComponentHit.AddDynamic(this, &AASGVehicle::OnHit);
    // GetMesh()->OnComponentBeginOverlap.AddDynamic(this, &AASGVehicle::OnBeginOverlap);
    AnnotationComponent->AddAnnotationColor(EAnnotationColor::Traversable, FLinearColor(0.f, 0.f, 0.f, 1.f));

    bEnabled = false;
    bWorldIsReady = false;
    TickNumber = 0;
    ResetTime = 0.f;
    NominalVehicleZLocation = 0.f;
    HeaderSequence = 1;
    PathSequence = 0; // Gets incremented when enabled

    ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	if (ROSInst)
	{
		// Create topic prefix
        FString TopicPrefix = FString::Printf(TEXT("/%s/"), *VehicleName);

        // Find ASG Worker ID, if applicable
		// TArray<AActor*> TempArray;
		// UGameplayStatics::GetAllActorsOfClass(GetWorld(), AASGWorker::StaticClass(), TempArray);
		// if (TempArray.Num() > 0)
		// {
		// 	AASGWorker* Worker = Cast<AASGWorker>(TempArray[0]);
		// 	if (Worker)
		// 	{
		// 		TopicPrefix = FString::Printf(TEXT("/asg_worker%i"), Worker->GetWorkerID()) + TopicPrefix;
		// 	}
		// } //UNCOMMENT

        EnableStatusPub =  NewObject<UTopic>(UTopic::StaticClass());

        FString StatusTopic = TopicPrefix + FString("enable_status");
		EnableStatusPub->Init(ROSInst->ROSIntegrationCore, StatusTopic, TEXT("std_msgs/Bool"));

		UE_LOG(LogTemp, Display, TEXT("Initialized evaluation vehicle ROS topic: %s"), *StatusTopic);
	}
}

void AASGVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
    Super::EndPlay(EndPlayReason);
    VehiclePath.poses.Empty();
    if (ROSInst)
    {
        EnableStatusPub->Unadvertise();
    }
}

void AASGVehicle::Tick(float DeltaTime) 
{
    Super::Tick(DeltaTime);
    TickNumber++;
    CheckIfReadyForEnable(DeltaTime);

    if (ROSInst)
    {
        TSharedPtr<ROSMessages::std_msgs::Bool> EnableStatusMessage(new ROSMessages::std_msgs::Bool(bEnabled));
        bool Success = EnableStatusPub->Publish(EnableStatusMessage);

        if (bEnabled)
        {
            // Add vehicle's location to VehiclePath. Covnert to conventional NWU coordinate frame.
            ROSMessages::geometry_msgs::PoseStamped PoseStamped;
            PoseStamped.header.seq = HeaderSequence;
            PoseStamped.header.time = FROSTime::Now();
            PoseStamped.header.frame_id = VehicleName;

            ROSMessages::geometry_msgs::Point Location(GetActorLocation()/100.f); // Put into [m]
            Location.y *= -1;
            PoseStamped.pose.position = Location;

            ROSMessages::geometry_msgs::Quaternion Quaternion;
            Quaternion = GetActorQuat();
            Quaternion.x *= -1;
            Quaternion.z *= -1;
            PoseStamped.pose.orientation = Quaternion;

            VehiclePath.poses.Emplace(PoseStamped);
            HeaderSequence++;
        }
    }
}

void AASGVehicle::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) 
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    if (DriveByWireComponent->IsManualDrive())
    {
        PlayerInputComponent->BindAxis(TEXT("DriveForward"), this, &AASGVehicle::DriveForward);
        PlayerInputComponent->BindAxis(TEXT("SteerRight"), this, &AASGVehicle::SteerRight);
        PlayerInputComponent->BindAction(TEXT("Handbrake"), IE_Pressed, this, &AASGVehicle::OnHandbrakePressed);
        PlayerInputComponent->BindAction(TEXT("Handbrake"), IE_Released, this, &AASGVehicle::OnHandbrakeReleased);
    }
}

FString AASGVehicle::GetVehicleName() const
{
    return VehicleName;
}  

void AASGVehicle::SetWorldIsReadyFlag(bool bReady)
{
    bWorldIsReady = bReady;
}

bool AASGVehicle::IsEnabled() const
{
    return bEnabled;
}

void AASGVehicle::SetDefaultResetInfo(FVector DefaultLocation, FRotator DefaultRotation)
{
    ResetLocation = DefaultLocation;
    ResetRotation = DefaultRotation;
}

float AASGVehicle::ResetVehicle(FVector NewLocation, FRotator NewRotation) 
{
    ResetLocation = NewLocation;
    ResetRotation = NewRotation;
    
    // float Performance = EvaluationComponent->ResetEvaluation();
    GetMesh()->SetAllPhysicsLinearVelocity(FVector(0.f));
    GetMesh()->SetAllPhysicsAngularVelocityInDegrees(FVector(0.f));
    DriveForward(0.f);
    SteerRight(0.f);

    SetActorLocationAndRotation(NewLocation, NewRotation, false, nullptr, ETeleportType::TeleportPhysics);
    bEnabled = false;
    bWorldIsReady = false;
    DriveByWireComponent->EnableDriveByWire(false);
    TickNumber = 0;
    ResetTime = 0.f;

    UE_LOG(LogTemp, Warning, TEXT("Vehicle has been reset to location %s and rotation %s."), *NewLocation.ToString(), *NewRotation.ToString());
    return 0.f; //Performance;
}

void AASGVehicle::DriveForward(float AxisValue) 
{
    if (bEnabled && bWorldIsReady)
    {
        // GetVehicleMovementComponent()->SetThrottleInput(AxisValue);
        DriveByWireComponent->SetDesiredForwardVelocity(AxisValue * DriveByWireComponent->GetMaxManualDriveSpeed());
    }
}

void AASGVehicle::SteerRight(float AxisValue) 
{
    if (bEnabled && bWorldIsReady)
    {
        // GetVehicleMovementComponent()->SetSteeringInput(AxisValue);
        DriveByWireComponent->SetDesiredSteeringAngle(AxisValue * DriveByWireComponent->GetMaxSteeringAngle());
    }
}

void AASGVehicle::OnHandbrakePressed() 
{
    if (bEnabled && bWorldIsReady)
    {
        // GetVehicleMovementComponent()->SetHandbrakeInput(true); // Engage handbrake
        DriveByWireComponent->SetHandbrakeInput(true);
    }
}

void AASGVehicle::OnHandbrakeReleased() 
{
    if (bEnabled && bWorldIsReady)
    {
        // GetVehicleMovementComponent()->SetHandbrakeInput(false); // Disengage handbrake
        DriveByWireComponent->SetHandbrakeInput(false);
    }
}

void AASGVehicle::CheckIfReadyForEnable(float DeltaTime) 
{
    // Wait at least a few ticks before checking vehicle velocity
    if (!bEnabled && bWorldIsReady && TickNumber > 5)
    {
        ResetTime += DeltaTime;
        if (ResetTime >= 5.f)
        {
            UE_LOG(LogTemp, Warning, TEXT("Vehicle did not reset in 5 sec, trying again."));
            ResetVehicle(ResetLocation, ResetRotation);
            bWorldIsReady = true;
            return;
        }

        if (FMath::Abs(GetActorRotation().Euler().X) > 15.f || FMath::Abs(GetActorRotation().Euler().Y) > 15.f)
        {
            UE_LOG(LogTemp, Warning, TEXT("Vehicle roll or pitch is too large after reset, trying again."));
            ResetVehicle(ResetLocation, ResetRotation);
            bWorldIsReady = true;
            return;
        }

        float TransVel = GetMesh()->GetPhysicsLinearVelocity().Size(); // [cm/s]
        float AngVel = GetMesh()->GetPhysicsAngularVelocityInDegrees().Size(); // [deg/s]
        float DistanceMoved = 0.f;

        if (CheckEnableTickNumber == 0)
        {
            CheckEnableLocation = GetActorLocation();
            CheckEnableTickNumber = TickNumber;
            return;
        }

        // Check distance moved after 10 ticks have passed
        if ((TickNumber - CheckEnableTickNumber) % 10 == 0 )
        {
            DistanceMoved = (GetActorLocation() - CheckEnableLocation).Size();
        }
        
        if (DistanceMoved <= 1.f && TransVel <= 10.f && AngVel <= 1.f)
        {
            bEnabled = true;
            CheckEnableTickNumber = 0;
            DriveByWireComponent->EnableDriveByWire(true);
            // EvaluationComponent->EnableEvaluation(true);
            
            VehiclePath.poses.Empty();
            VehiclePath.header.frame_id = VehicleName;
            VehiclePath.header.time = FROSTime::Now();
            VehiclePath.header.seq = ++PathSequence;
            HeaderSequence = 1;

            NominalVehicleZLocation = GetActorLocation().Z;
            UE_LOG(LogTemp, Warning, TEXT("Vehicle is enabled."));
        }
        else
        {
            CheckEnableLocation = GetActorLocation();
            CheckEnableTickNumber = TickNumber;
        }
    }
}

float AASGVehicle::GetNominalVehicleZLocation() 
{
    return NominalVehicleZLocation;
}

void AASGVehicle::GetVehiclePath(class ROSMessages::nav_msgs::Path &Path) 
{
    Path = VehiclePath;
}

void AASGVehicle::OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, 
				FVector NormalImpulse, const FHitResult& Hit) 
{
    if (!bEnabled) return;
    
    AStructuralSceneActor* SSAActor = Cast<AStructuralSceneActor>(OtherActor);
    if (SSAActor)
    {
        // EvaluationComponent->HitStructuralSceneActor();
        UE_LOG(LogTemp, Warning, TEXT("Vehicle hit a structural scene actor."));
    }
}

// void AASGVehicle::OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, 
// 				UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult &SweepResult)
// {
//     UE_LOG(LogTemp, Warning, TEXT("Vehicle overlapped something."));
// }