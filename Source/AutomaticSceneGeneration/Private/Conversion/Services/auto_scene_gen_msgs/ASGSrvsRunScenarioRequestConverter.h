// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Services/BaseRequestConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/geometry_msgs/GeometryMsgsPointConverter.h"
#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsStructuralSceneActorLayoutConverter.h"
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

		request->done_testing = UBaseMessageConverter::GetBoolFromBSON(key + ".done_testing", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->scenario_number = UBaseMessageConverter::GetInt32FromBSON(key + ".scenario_number", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		KeyFound = UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".vehicle_start_location", &request->vehicle_start_location, LogOnErrors); if (!KeyFound) return false;
		request->vehicle_start_yaw = UBaseMessageConverter::GetDoubleFromBSON(key + ".vehicle_start_yaw", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		KeyFound = UGeometryMsgsPointConverter::_bson_extract_child_point(b, key + ".vehicle_goal_location", &request->vehicle_goal_location, LogOnErrors); if (!KeyFound) return false;
		request->goal_radius = UBaseMessageConverter::GetDoubleFromBSON(key + ".goal_radius", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		request->ssa_array = UBaseMessageConverter::GetTArrayFromBSON<ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout>(key + ".ssa_array", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout elem;
			subKeyFound = UASGMsgsStructuralSceneActorLayoutConverter::_bson_extract_child_msg(subMsg, subKey, &elem, LogOnErrors);
			return elem;
		}, LogOnErrors);
		if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_request(bson_t *b, const ROSMessages::auto_scene_gen_msgs::FRunScenarioRequest *request)
	{
		BSON_APPEND_BOOL(b, "done_testing", request->done_testing);
		BSON_APPEND_INT32(b, "scenario_number", request->scenario_number);

		UGeometryMsgsPointConverter::_bson_append_child_point(b, "vehicle_start_location", &request->vehicle_start_location);
		BSON_APPEND_DOUBLE(b, "vehicle_start_yaw", request->vehicle_start_yaw);
		
		UGeometryMsgsPointConverter::_bson_append_child_point(b, "vehicle_goal_location", &request->vehicle_goal_location);
		BSON_APPEND_DOUBLE(b, "goal_radius", request->goal_radius);

		UBaseMessageConverter::_bson_append_tarray<ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout>(b, "ssa_array", request->ssa_array, [](bson_t* msg, const char* key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout &ssa_info)
		{
			UASGMsgsStructuralSceneActorLayoutConverter::_bson_append_child_msg(msg, key, &ssa_info);
		});
	}
	
};
