// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseResponseConverter.h"
#include "auto_scene_gen_msgs/srv/SceneCaptureResponse.h"
#include "ASGSrvsSceneCaptureResponseConverter.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API UASGSrvsSceneCaptureResponseConverter : public UBaseResponseConverter
{
	GENERATED_BODY()

public:
	UASGSrvsSceneCaptureResponseConverter();

	virtual bool ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) override;
	virtual bool ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) override;

	virtual TSharedPtr<FROSBaseServiceResponse> AllocateConcreteResponse() override;

	static bool _bson_extract_child_response(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::FSceneCaptureResponse *response, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		response->received = UBaseMessageConverter::GetBoolFromBSON(key + ".received", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_response(bson_t *b, const ROSMessages::auto_scene_gen_msgs::FSceneCaptureResponse *response)
	{
		BSON_APPEND_BOOL(b, "received", response->received);
	}
};
