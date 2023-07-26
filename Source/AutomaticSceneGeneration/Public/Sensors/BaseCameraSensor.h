// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/SceneCaptureComponent2D.h"
#include "BaseCameraSensor.generated.h"

// Struct for queuing frame data
struct FFrameData
{
    TArray<FColor> ImageData;
    FString Filename;

	FFrameData(const TArray<FColor> ImageDataIn, const FString& FilenameIn) 
	{
		ImageData = ImageDataIn;
		Filename = FilenameIn;
	}
};

// Base class
UCLASS()
class AUTOMATICSCENEGENERATION_API UBaseCameraSensor : public USceneCaptureComponent2D
{
	GENERATED_BODY()
public:
	UBaseCameraSensor();

protected: /****************************** USceneCaptureComponent2D Overrides ******************************/
	// Called when the game starts
	virtual void BeginPlay() override;

public: /****************************** USceneCaptureComponent2D Overrides ******************************/
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
public: /****************************** UBaseCameraSensor ******************************/
	int32 GetImageWidth();

	int32 GetImageHeight();

	void SetSaveImages(bool bSaveImagesIn);

	void SetSavePrefix(FString NewSavePrefix);

	int32 GetFrameNumber() const;

	void SetFrameNumber(int32 NewFrameNumber);

	/**
	 * Initializes a new texture target. This must be called before using the camera.
	 * @param NewWidth The new frame width in pixels
	 * @param NewHeight The new frame height in pixels
	 * @param NewFOV Horizontal field of view in degrees
	 */
	virtual void InitTextureTarget(int32 NewWidth, int32 NewHeight, float NewFOV);

	/**
	 * Resize the texture target width and height
	 * @param NewWidth The new frame width in pixels
	 * @param NewHeight The new frame height in pixels
	 */
	void ResizeTextureTarget(int32 NewWidth, int32 NewHeight);

	/**
	 * Set the camera FOV
	 * @param NewFOV The new FOV angle in [deg]
	*/
	void SetFOV(float NewFOV);

	/**
	 * Captures an image of the scene based on the FColor format (used for most cameras). Make sure your texture render target is set correctly before using this.
	 * @param ImageData The TArray that the render thread will output the scene capture data to.
	 * @param bUseAnnotationMaterial Indicates if an annotation material should be used when capturing the image. If false, then the actor's default/regular material will be used.
	 * @param AnnotationMaterialID Indicates the type of annotationa material to use when capturing the image. Only takes effect if bUseAnnotationMaterial is true.
	 */
	virtual void CaptureColor(TArray<FColor> &ImageData, bool bUseAnnotationMaterial = false, uint8 AnnotationMaterialID = 0);

	/**
	 * Captures an image of the scene based on the FFloat16Color format (used by a depth camera). Make sure your texture render target is set correctly before using this.
	 * @param ImageData The TArray that the render thread will output the scene capture data to.
	 */
	virtual void CaptureFloat16Color(TArray<FFloat16Color> &ImageData);

protected: /****************************** UBaseCameraSensor ******************************/
	int32 ImageWidth = 640;
	int32 ImageHeight = 480;
	bool bInitTextureTarget = false;

	// Used for saving frames to disk
	bool bSaveImages = false;
	FString SavePrefix = "";
	int32 FrameNumber = 1;
	TQueue<FFrameData*> PendingSaveQueue;
	TSharedPtr<class IImageWrapper> ImageWrapper;

	// Used by CaptureColor() function
	TArray<class UAnnotationComponent*> WorldAnnotationComponents;

	// Gathers pointers to all UAnnotationComponents in the world.
	virtual void InitAnnotatableActors();

	void SavePendingFrames();
};

/**
 * This class defines an asynchronous task for saving the frame data to disk
 */
class AsyncSaveImageTask : public FNonAbandonableTask
{
public:
	AsyncSaveImageTask(TArray64<uint8> ImageIn, FString FilenameIn)
	{
		Image = ImageIn;
		Filename = FilenameIn;
	}
	~AsyncSaveImageTask() {}

	// Required by UE4!
	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(AsyncSaveImageTask, STATGROUP_ThreadPoolAsyncTasks);
	}

protected:
    TArray64<uint8> Image;
    FString Filename = "";

public:
    void DoWork()
	{
		FFileHelper::SaveArrayToFile(Image, *Filename);
    	// UE_LOG(LogTemp, Warning, TEXT("Saved Image: %s"), *Filename);
	}
};
