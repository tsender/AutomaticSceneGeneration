// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "auto_scene_gen_msgs/StructuralSceneAttributes.h"
#include "ASGMsgsStructuralSceneAttributesConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsStructuralSceneAttributesConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsStructuralSceneAttributesConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->visible = GetBoolFromBSON(key + ".visible", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->x = GetDoubleFromBSON(key + ".x", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->y = GetDoubleFromBSON(key + ".y", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->yaw = GetDoubleFromBSON(key + ".yaw", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->scale = GetDoubleFromBSON(key + ".scale", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::StructuralSceneAttributes *msg)
	{
		BSON_APPEND_BOOL(b, "visible", msg->visible);
		BSON_APPEND_DOUBLE(b, "x", msg->x);
		BSON_APPEND_DOUBLE(b, "y", msg->y);
		BSON_APPEND_DOUBLE(b, "yaw", msg->yaw);
		BSON_APPEND_DOUBLE(b, "scale", msg->scale);
	}
};
