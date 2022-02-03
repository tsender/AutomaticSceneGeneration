// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "auto_scene_gen_msgs/msg/StructuralSceneActorLayout.h"
#include "ASGMsgsStructuralSceneActorLayoutConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsStructuralSceneActorLayoutConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsStructuralSceneActorLayoutConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->path_name = GetFStringFromBSON(key + ".path_name", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->num_instances = GetInt32FromBSON(key + ".num_instances", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		msg->visibilities = GetBoolTArrayFromBSON(key + ".visibilities", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->x_positions = GetFloatTArrayFromBSON(key + ".x_positions", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->y_positions = GetFloatTArrayFromBSON(key + ".y_positions", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->yaw_angles = GetFloatTArrayFromBSON(key + ".yaw_angles", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->scale_factors = GetFloatTArrayFromBSON(key + ".scale_factors", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout *msg)
	{
		BSON_APPEND_UTF8(b, "path_name", TCHAR_TO_UTF8(*msg->path_name));
		BSON_APPEND_INT32(b, "num_instances", msg->num_instances);

		_bson_append_bool_tarray(b, "visibilities", msg->visibilities);
		_bson_append_float_tarray(b, "x_positions", msg->x_positions);
		_bson_append_float_tarray(b, "y_positions", msg->y_positions);
		_bson_append_float_tarray(b, "yaw_angles", msg->yaw_angles);
		_bson_append_float_tarray(b, "scale_factors", msg->scale_factors);
	}
};
