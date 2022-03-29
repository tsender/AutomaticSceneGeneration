// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsStructuralSceneActorLayoutConverter.h"
#include "auto_scene_gen_msgs/msg/SceneDescription.h"
#include "ASGMsgsSceneDescriptionConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsSceneDescriptionConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsSceneDescriptionConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::SceneDescription *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->ssa_array = GetTArrayFromBSON<ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout>(key + ".ssa_array", b, KeyFound, [LogOnErrors](FString subKey, bson_t* subMsg, bool& subKeyFound)
		{
			ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout elem;
			subKeyFound = UASGMsgsStructuralSceneActorLayoutConverter::_bson_extract_child_msg(subMsg, subKey, &elem, LogOnErrors);
			return elem;
		}, LogOnErrors);
		if (!KeyFound) return false;
		
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::SceneDescription *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::SceneDescription *msg)
	{
		_bson_append_tarray<ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout>(b, "ssa_array", msg->ssa_array, [](bson_t* msg, const char* key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneActorLayout &ssa_layout)
		{
			UASGMsgsStructuralSceneActorLayoutConverter::_bson_append_child_msg(msg, key, &ssa_layout);
		});
	}
};
