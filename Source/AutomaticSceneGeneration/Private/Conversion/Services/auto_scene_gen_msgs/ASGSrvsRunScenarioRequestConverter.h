// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Services/BaseRequestConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"
#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsSceneDescriptionConverter.h"
#include "auto_scene_gen_msgs/srv/RunScenarioRequest.h"
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

	static bool _bson_extract_child_request(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::FRunScenarioRequest *request, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		request->scenario_number = UBaseMessageConverter::GetInt32FromBSON(key + ".scenario_number", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		request->sim_timeout_period = UBaseMessageConverter::GetDoubleFromBSON(key + ".sim_timeout_period", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->vehicle_idling_timeout_period = UBaseMessageConverter::GetDoubleFromBSON(key + ".vehicle_idling_timeout_period", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->vehicle_stuck_timeout_period = UBaseMessageConverter::GetDoubleFromBSON(key + ".vehicle_stuck_timeout_period", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->allow_collisions = UBaseMessageConverter::GetBoolFromBSON(key + ".allow_collisions", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		if (!UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".vehicle_start_location", &request->vehicle_start_location, LogOnErrors)) return false;
		request->vehicle_start_yaw = UBaseMessageConverter::GetDoubleFromBSON(key + ".vehicle_start_yaw", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		if (!UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".vehicle_goal_location", &request->vehicle_goal_location, LogOnErrors)) return false;
		request->goal_radius = UBaseMessageConverter::GetDoubleFromBSON(key + ".goal_radius", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		if (!UASGMsgsSceneDescriptionConverter::_bson_extract_child_msg(b, key + ".scene_description", &request->scene_description, LogOnErrors)) return false;

		return true;
	}

	static void _bson_append_request(bson_t *b, const ROSMessages::auto_scene_gen_msgs::FRunScenarioRequest *request)
	{
		BSON_APPEND_INT32(b, "scenario_number", request->scenario_number);

		BSON_APPEND_DOUBLE(b, "sim_timeout_period", request->sim_timeout_period);
		BSON_APPEND_DOUBLE(b, "vehicle_idling_timeout_period", request->vehicle_idling_timeout_period);
		BSON_APPEND_DOUBLE(b, "vehicle_stuck_timeout_period", request->vehicle_stuck_timeout_period);
		BSON_APPEND_BOOL(b, "allow_collisions", request->allow_collisions);

		UGeometryMsgsPointConverter::_bson_append_child_point(b, "vehicle_start_location", &request->vehicle_start_location);
		BSON_APPEND_DOUBLE(b, "vehicle_start_yaw", request->vehicle_start_yaw);
		
		UGeometryMsgsPointConverter::_bson_append_child_point(b, "vehicle_goal_location", &request->vehicle_goal_location);
		BSON_APPEND_DOUBLE(b, "goal_radius", request->goal_radius);

		UASGMsgsSceneDescriptionConverter::_bson_append_child_msg(b, "scene_description", &request->scene_description);
	}
	
};
