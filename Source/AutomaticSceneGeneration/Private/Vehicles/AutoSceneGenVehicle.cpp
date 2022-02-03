// Fill out your copyright notice in the Description page of Project Settings.


#include "Vehicles/AutoSceneGenVehicle.h"
#include "Components/PIDDriveByWireComponent.h"
#include "Components/AnnotationComponent.h"
#include "Components/AudioComponent.h"
#include "WheeledVehicleMovementComponent.h"
#include "Actors/StructuralSceneActor.h"
#include "Actors/AutoSceneGenWorker.h"
#include "Sound/SoundCue.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"
#include "auto_scene_gen_logging.h"

#include "ROSIntegration/Classes/ROSIntegrationGameInstance.h"
#include "ROSIntegration/Classes/RI/Topic.h"
#include "ROSIntegration/Public/ROSTime.h"
#include "ROSIntegration/Public/std_msgs/Bool.h"
#include "ROSIntegration/Public/nav_msgs/Path.h"


AAutoSceneGenVehicle::AAutoSceneGenVehicle() 
{
    DriveByWireComponent = CreateDefaultSubobject<UPIDDriveByWireComponent>(TEXT("PID Drive By Wire"));
    AnnotationComponent = CreateDefaultSubobject<UAnnotationComponent>(TEXT("Annotation Component"));

    static ConstructorHelpers::FObjectFinder<USoundCue> SoundCue(TEXT("/Game/VehicleAdv/Sound/Engine_Loop_Cue.Engine_Loop_Cue"));
	EngineSoundComponent = CreateDefaultSubobject<UAudioComponent>(TEXT("Engine Sound"));
	EngineSoundComponent->SetSound(SoundCue.Object);
	EngineSoundComponent->SetupAttachment(GetMesh());
}

void AAutoSceneGenVehicle::BeginPlay() 
{
    Super::BeginPlay();

    // Make sure skeletal mesh is set to generate hit events
    GetMesh()->OnComponentHit.AddDynamic(this, &AAutoSceneGenVehicle::OnHit);
    // GetMesh()->OnComponentBeginOverlap.AddDynamic(this, &AAutoSceneGenVehicle::OnBeginOverlap);
    AnnotationComponent->AddAnnotationColor(EAnnotationColor::Traversable, FColor(0, 0, 0, 255));

    bEnabled = false;
    bWorldIsReady = false;
    TickNumber = 0;
    ResetTime = 0.f;
    NominalVehicleZLocation = 0.f;
    HeaderSequence = 1;
    PathSequence = 0; // Gets incremented when enabled
    NumSSAHit = 0;

    // Start with handbrake engaged
    DriveByWireComponent->SetHandbrakeInput(true);

    ROSInst = Cast<UROSIntegrationGameInstance>(GetGameInstance());
	if (ROSInst)
	{
		// Create topic prefix
        FString TopicPrefix = FString::Printf(TEXT("/%s/"), *VehicleName);

        // Find AutoSceneGen Worker ID, if applicable
		TArray<AActor*> TempArray;
		UGameplayStatics::GetAllActorsOfClass(GetWorld(), AAutoSceneGenWorker::StaticClass(), TempArray);
		if (TempArray.Num() > 0)
		{
			AAutoSceneGenWorker* Worker = Cast<AAutoSceneGenWorker>(TempArray[0]);
			if (Worker)
			{
				TopicPrefix = FString::Printf(TEXT("/asg_worker%i"), Worker->GetWorkerID()) + TopicPrefix;
			}
		}

        EnableStatusPub =  NewObject<UTopic>(UTopic::StaticClass());

        FString StatusTopic = TopicPrefix + FString("enable_status");
		EnableStatusPub->Init(ROSInst->ROSIntegrationCore, StatusTopic, TEXT("std_msgs/Bool"));
        EnableStatusPub->Advertise();
		UE_LOG(LogASG, Display, TEXT("Initialized evaluation vehicle ROS topic: %s"), *StatusTopic);
	}
}

void AAutoSceneGenVehicle::EndPlay(const EEndPlayReason::Type EndPlayReason) 
{
    Super::EndPlay(EndPlayReason);
    VehiclePath.poses.Empty();
    if (ROSInst)
    {
        EnableStatusPub->Unadvertise();
    }
}

void AAutoSceneGenVehicle::Tick(float DeltaTime) 
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

void AAutoSceneGenVehicle::SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) 
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    if (DriveByWireComponent->IsManualDrive())
    {
        PlayerInputComponent->BindAxis(TEXT("DriveForward"), this, &AAutoSceneGenVehicle::DriveForward);
        PlayerInputComponent->BindAxis(TEXT("SteerRight"), this, &AAutoSceneGenVehicle::SteerRight);
        PlayerInputComponent->BindAction(TEXT("Handbrake"), IE_Pressed, this, &AAutoSceneGenVehicle::OnHandbrakePressed);
        PlayerInputComponent->BindAction(TEXT("Handbrake"), IE_Released, this, &AAutoSceneGenVehicle::OnHandbrakeReleased);
    }
}

FString AAutoSceneGenVehicle::GetVehicleName() const
{
    return VehicleName;
}  

void AAutoSceneGenVehicle::SetWorldIsReadyFlag(bool bReady)
{
    bWorldIsReady = bReady;
}

bool AAutoSceneGenVehicle::IsEnabled() const
{
    return bEnabled;
}

void AAutoSceneGenVehicle::SetDefaultResetInfo(FVector DefaultLocation, FRotator DefaultRotation)
{
    ResetLocation = DefaultLocation;
    ResetRotation = DefaultRotation;
}

void AAutoSceneGenVehicle::ResetVehicle(FVector NewLocation, FRotator NewRotation) 
{
    ResetLocation = NewLocation;
    ResetRotation = NewRotation;
    
    GetMesh()->SetAllPhysicsLinearVelocity(FVector(0.f));
    GetMesh()->SetAllPhysicsAngularVelocityInDegrees(FVector(0.f));
    DriveForward(0.f);
    SteerRight(0.f);
    DriveByWireComponent->SetHandbrakeInput(true);

    SetActorLocationAndRotation(NewLocation, NewRotation, false, nullptr, ETeleportType::TeleportPhysics);
    bEnabled = false;
    bWorldIsReady = false;
    DriveByWireComponent->EnableDriveByWire(false);
    TickNumber = 0;
    ResetTime = 0.f;

    UE_LOG(LogASG, Display, TEXT("Vehicle has been reset to location %s and rotation %s."), *NewLocation.ToString(), *NewRotation.ToString());
}

void AAutoSceneGenVehicle::ResetVehicle(FVector NewLocation, FRotator NewRotation, ROSMessages::nav_msgs::Path &Path) 
{
    Path = VehiclePath;
    ResetVehicle(NewLocation, NewRotation);
}

void AAutoSceneGenVehicle::DriveForward(float AxisValue) 
{
    if (bEnabled && bWorldIsReady)
    {
        DriveByWireComponent->SetDesiredForwardVelocity(AxisValue * DriveByWireComponent->GetMaxManualDriveSpeed());
    }
}

void AAutoSceneGenVehicle::SteerRight(float AxisValue) 
{
    if (bEnabled && bWorldIsReady)
    {
        DriveByWireComponent->SetDesiredSteeringAngle(AxisValue * DriveByWireComponent->GetMaxSteeringAngle());
    }
}

void AAutoSceneGenVehicle::OnHandbrakePressed() 
{
    if (bEnabled && bWorldIsReady)
    {
        DriveByWireComponent->SetHandbrakeInput(true); // Engage handbrake
    }
}

void AAutoSceneGenVehicle::OnHandbrakeReleased() 
{
    if (bEnabled && bWorldIsReady)
    {
        DriveByWireComponent->SetHandbrakeInput(false); // Disengage handbrake
    }
}

void AAutoSceneGenVehicle::CheckIfReadyForEnable(float DeltaTime) 
{
    // Wait at least a few ticks before checking vehicle velocity
    if (!bEnabled && bWorldIsReady && TickNumber > 5)
    {
        // Make sure vehicle reset within 5 sec
        // If rendering takes too long, then this might become an issue and will need a workaround
        ResetTime += DeltaTime;
        if (ResetTime >= 5.f)
        {
            UE_LOG(LogASG, Warning, TEXT("Vehicle did not reset in 5 sec, trying again."));
            ResetVehicle(ResetLocation, ResetRotation);
            bWorldIsReady = true;
            return;
        }

        // If vehicle roll/pitch is too large after reset (e.g. from being thrown out of bounds), then reset the vehicle to try again
        if (FMath::Abs(GetActorRotation().Euler().X) > 15.f || FMath::Abs(GetActorRotation().Euler().Y) > 15.f)
        {
            UE_LOG(LogASG, Warning, TEXT("Vehicle roll or pitch is too large after reset, trying again."));
            ResetVehicle(ResetLocation, ResetRotation);
            bWorldIsReady = true;
            return;
        }

        float TransVel = GetMesh()->GetPhysicsLinearVelocity().Size(); // Translational speed [cm/s]
        float AngVel = GetMesh()->GetPhysicsAngularVelocityInDegrees().Size(); // Angular speed [deg/s]
        float DistanceMoved = 0.f;

        if (CheckEnableTickNumber == 0)
        {
            CheckEnableLocation = GetActorLocation();
            CheckEnableTickNumber = TickNumber;
            return;
        }

        // Check distance traveled over 10 ticks
        if ((TickNumber - CheckEnableTickNumber) % 10 == 0 )
        {
            DistanceMoved = (GetActorLocation() - CheckEnableLocation).Size();
        }
        
        // If criteria below is met, then we can assume the vehicle was reset properly without issue
        if (DistanceMoved <= 1.f && TransVel <= 10.f && AngVel <= 1.f)
        {
            bEnabled = true;
            CheckEnableTickNumber = 0;
            DriveByWireComponent->EnableDriveByWire(true);
            
            // Reset the internal VehiclePath message here
            VehiclePath.poses.Empty();
            VehiclePath.header.frame_id = VehicleName;
            VehiclePath.header.time = FROSTime::Now();
            VehiclePath.header.seq = ++PathSequence;
            HeaderSequence = 1;

            NominalVehicleZLocation = GetActorLocation().Z;
            UE_LOG(LogASG, Display, TEXT("Vehicle is enabled."));
        }
        else
        {
            CheckEnableLocation = GetActorLocation();
            CheckEnableTickNumber = TickNumber;
        }
    }
}

float AAutoSceneGenVehicle::GetNominalVehicleZLocation() 
{
    return NominalVehicleZLocation;
}

void AAutoSceneGenVehicle::GetVehiclePath(class ROSMessages::nav_msgs::Path &Path) 
{
    Path = VehiclePath;
}

int32 AAutoSceneGenVehicle::GetNumStructuralSceneActorsHit() const
{
    return NumSSAHit;
}

void AAutoSceneGenVehicle::OnHit(UPrimitiveComponent* HitComp, AActor* OtherActor, UPrimitiveComponent* OtherComp, 
				FVector NormalImpulse, const FHitResult& Hit) 
{
    if (!bEnabled) return;
    
    AStructuralSceneActor* SSAActor = Cast<AStructuralSceneActor>(OtherActor);
    if (SSAActor && SSAActor->IsTraversable())
    {
        NumSSAHit++;
        UE_LOG(LogASG, Warning, TEXT("Vehicle hit structural scene actor %s."), *SSAActor->GetName());
    }
}

// void AAutoSceneGenVehicle::OnBeginOverlap(UPrimitiveComponent* OverlappedComponent, AActor* OtherActor, 
// 				UPrimitiveComponent* OtherComp, int32 OtherBodyIndex, bool bFromSweep, const FHitResult &SweepResult)
// {
//     UE_LOG(LogASG, Warning, TEXT("Vehicle overlapped something."));
// }