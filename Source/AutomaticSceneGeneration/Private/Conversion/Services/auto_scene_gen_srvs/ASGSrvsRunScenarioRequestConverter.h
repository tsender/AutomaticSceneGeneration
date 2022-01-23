// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/std_msgs/StdMsgsFloat32MultiArrayConverter.h"
#include "auto_scene_gen_srvs/RunScenarioRequest.h"
#include "ASGSrvsRunScenarioRequestConverter.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API UASGSrvsRunScenarioRequestConverter : public UBaseRequestConverter
{
	GENERATED_BODY()

public:
	UASGSrvsRunScenarioRequestConverter();

	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) override;
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) override;

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest() override;

	static bool _bson_extract_child_request(bson_t *b, FString key, auto_scene_gen_srvs::FRunScenarioRequest *request, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		request->done_testing = UBaseMessageConverter::GetBoolFromBSON(key + ".done_testing", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->scenario_number = UBaseMessageConverter::GetInt32FromBSON(key + ".scenario_number", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		bool bSuccess = UStdMsgsFloat32MultiArrayConverter::_bson_extract_child_float_multi_array(b, key + ".ssa_array", &request->ssa_array, LogOnErrors);
		return bSuccess;
	}

	static void _bson_append_request(bson_t *b, const auto_scene_gen_srvs::FRunScenarioRequest *request)
	{
		BSON_APPEND_BOOL(b, "done_testing", request->done_testing);
		BSON_APPEND_INT32(b, "scenario_number", request->scenario_number);
		UStdMsgsFloat32MultiArrayConverter::_bson_append_child_float_multi_array(b, "ssa_array", &request->ssa_array);
	}
	
};
