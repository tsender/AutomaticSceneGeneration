// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsStructuralSceneAttributesConverter.h"
#include "auto_scene_gen_msgs/StructuralSceneActorArray.h"
#include "ASGMsgsStructuralSceneActorArrayConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsStructuralSceneActorArrayConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsStructuralSceneActorArrayConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::StructuralSceneActorArray *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->path_name = GetFStringFromBSON(key + ".path_name", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		msg->attr_array = GetTArrayFromBSON<ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes>(key + ".attr_array", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes elem;
			subKeyFound = UASGMsgsStructuralSceneAttributesConverter::_bson_extract_child_msg(subMsg, subKey, &elem, LogOnErrors);
			return elem;
		}, LogOnErrors);
		if (!KeyFound) return false;
		
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorArray *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorArray *msg)
	{
		BSON_APPEND_UTF8(b, "path_name", TCHAR_TO_UTF8(*msg->path_name));
		_bson_append_tarray<ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes>(b, "attr_array", msg->attr_array, [](bson_t* msg, const char* key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes &attr)
		{
			UASGMsgsStructuralSceneAttributesConverter::_bson_append_child_msg(msg, key, &attr);
		});
	}
};
