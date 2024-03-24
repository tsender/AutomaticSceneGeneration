// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/sensor_msgs/SensorMsgsImageConverter.h"
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

	static bool _bson_extract_child_request(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::AnalyzeScenarioRequest *request, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		request->worker_id = UBaseMessageConverter::GetInt32FromBSON(key + ".worker_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->scenario_number = UBaseMessageConverter::GetInt32FromBSON(key + ".scenario_number", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		request->scene_capture_only = UBaseMessageConverter::GetBoolFromBSON(key + ".scene_capture_only", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->scene_captures = GetTArrayFromBSON<ROSMessages::sensor_msgs::Image>(key + ".scene_captures", b, KeyFound, [](FString subKey, bson_t* subMsg, bool& subKeyFound) {
			ROSMessages::sensor_msgs::Image ret;
			subKeyFound = USensorMsgsImageConverter::_bson_extract_child_image(subMsg, subKey, &ret);
			return ret;
		});
		if (!KeyFound) return false;
		request->scene_capture_names = GetFStringTArrayFromBSON(key + ".scene_capture_names", b, KeyFound); if (!KeyFound) return false;

		request->termination_reason = UBaseMessageConverter::GetInt32FromBSON(key + ".termination_reason", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		request->vehicle_trajectory = UBaseMessageConverter::GetTArrayFromBSON<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance>(key + ".vehicle_trajectory", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance ret;
			subKeyFound = UASGMsgsOdometryWithoutCovarianceConverter::_bson_extract_child_msg(subMsg, subKey, &ret, LogOnErrors);
			return ret;
		}, LogOnErrors);
		if (!KeyFound) return false;
		
		request->vehicle_sim_time = UBaseMessageConverter::GetDoubleFromBSON(key + ".vehicle_sim_time", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->num_vehicle_control_messages = UBaseMessageConverter::GetInt32FromBSON(key + ".num_vehicle_control_messages", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_request(bson_t *b, const ROSMessages::auto_scene_gen_msgs::AnalyzeScenarioRequest *request)
	{
		BSON_APPEND_INT32(b, "worker_id", request->worker_id);
		BSON_APPEND_INT32(b, "scenario_number", request->scenario_number);

		BSON_APPEND_BOOL(b, "scene_capture_only", request->scene_capture_only);
		UBaseMessageConverter::_bson_append_tarray<ROSMessages::sensor_msgs::Image>(b, "scene_captures", request->scene_captures, [](bson_t* msg, const char* key, const ROSMessages::sensor_msgs::Image &image)
		{
			USensorMsgsImageConverter::_bson_append_child_image(msg, key, &image);
		});
		_bson_append_fstring_tarray(b, "scene_capture_names", request->scene_capture_names);

		BSON_APPEND_INT32(b, "termination_reason", request->termination_reason);

		UBaseMessageConverter::_bson_append_tarray<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance>(b, "vehicle_trajectory", request->vehicle_trajectory, [](bson_t* msg, const char* key, const ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance& odometry)
		{
			UASGMsgsOdometryWithoutCovarianceConverter::_bson_append_child_msg(msg, key, &odometry);
		});

		BSON_APPEND_DOUBLE(b, "vehicle_sim_time", request->vehicle_sim_time);
		BSON_APPEND_INT32(b, "num_vehicle_control_messages", request->num_vehicle_control_messages);
	}
};
