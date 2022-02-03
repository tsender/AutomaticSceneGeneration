// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "Conversion/Messages/nav_msgs/NavMsgsPathConverter.h"
#include "auto_scene_gen_msgs/srv/AnalyzeScenarioRequest.h"
#include "ASGSrvsAnalyzeScenarioRequestConverter.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API UASGSrvsAnalyzeScenarioRequestConverter : public UBaseRequestConverter
{
	GENERATED_BODY()

public:
	UASGSrvsAnalyzeScenarioRequestConverter();

	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) override;
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) override;

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest() override;

	static bool _bson_extract_child_request(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest *request, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		request->worker_id = UBaseMessageConverter::GetInt32FromBSON(key + ".worker_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->scenario_number = UBaseMessageConverter::GetInt32FromBSON(key + ".scenario_number", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->crashed = UBaseMessageConverter::GetBoolFromBSON(key + ".crashed", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->succeeded = UBaseMessageConverter::GetBoolFromBSON(key + ".succeeded", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		bool bSuccess = UNavMsgsPathConverter::_bson_extract_child_path(b, key + ".vehicle_path", &request->vehicle_path, LogOnErrors);
		return bSuccess;
	}

	static void _bson_append_request(bson_t *b, const ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest *request)
	{
		BSON_APPEND_INT32(b, "worker_id", request->worker_id);
		BSON_APPEND_INT32(b, "scenario_number", request->scenario_number);
		BSON_APPEND_BOOL(b, "crashed", request->crashed);
		BSON_APPEND_BOOL(b, "succeeded", request->succeeded);
		UNavMsgsPathConverter::_bson_append_child_path(b, "vehicle_path", &request->vehicle_path);
	}
	
};
