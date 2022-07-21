// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsOdometryWithoutCovarianceConverter.h"
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
		request->termination_reason = UBaseMessageConverter::GetInt32FromBSON(key + ".termination_reason", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		request->vehicle_trajectory = UBaseMessageConverter::GetTArrayFromBSON<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance>(key + ".poses", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance ret;
			subKeyFound = UASGMsgsOdometryWithoutCovarianceConverter::_bson_extract_child_msg(subMsg, subKey, &ret, LogOnErrors);
			return ret;
		}, LogOnErrors);
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_request(bson_t *b, const ROSMessages::auto_scene_gen_msgs::FAnalyzeScenarioRequest *request)
	{
		BSON_APPEND_INT32(b, "worker_id", request->worker_id);
		BSON_APPEND_INT32(b, "scenario_number", request->scenario_number);
		BSON_APPEND_INT32(b, "termination_reason", request->termination_reason);

		UBaseMessageConverter::_bson_append_tarray<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance>(b, "vehicle_trajectory", request->vehicle_trajectory, [](bson_t* msg, const char* key, const ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance& odometry)
		{
			UASGMsgsOdometryWithoutCovarianceConverter::_bson_append_child_msg(msg, key, &odometry);
		});
	}
	
};
