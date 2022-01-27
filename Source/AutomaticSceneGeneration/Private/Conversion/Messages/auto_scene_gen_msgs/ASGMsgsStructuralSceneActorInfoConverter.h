// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/std_msgs/StdMsgsFloat32MultiArrayConverter.h"
#include "auto_scene_gen_msgs/StructuralSceneActorInfo.h"
#include "ASGMsgsStructuralSceneActorInfoConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsStructuralSceneActorInfoConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsStructuralSceneActorInfoConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::StructuralSceneActorInfo *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->path_name = GetFStringFromBSON(key + ".path_name", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->count = GetInt32FromBSON(key + ".count", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		bool bSuccess = UStdMsgsFloat32MultiArrayConverter::_bson_extract_child_float_multi_array(b, key + ".attr_array", &msg->attr_array, LogOnErrors);
		
		return bSuccess;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorInfo *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorInfo *msg)
	{
		BSON_APPEND_UTF8(b, "path_name", TCHAR_TO_UTF8(*msg->path_name));
		BSON_APPEND_INT32(b, "count", msg->count);
		UStdMsgsFloat32MultiArrayConverter::_bson_append_child_float_multi_array(b, "attr_array", &msg->attr_array);
	}
};
