// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "auto_scene_gen_msgs/msg/LandscapeDescription.h"
#include "ASGMsgsLandscapeDescriptionConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsLandscapeDescriptionConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsLandscapeDescriptionConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::LandscapeDescription *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->nominal_size = GetDoubleFromBSON(key + ".nominal_size", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->subdivisions = GetInt32FromBSON(key + ".subdivisions", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->border = GetDoubleFromBSON(key + ".border", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::LandscapeDescription *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::LandscapeDescription *msg)
	{
		BSON_APPEND_DOUBLE(b, "nominal_size", msg->nominal_size);
		BSON_APPEND_INT32(b, "subdivisions", msg->subdivisions);
		BSON_APPEND_DOUBLE(b, "border", msg->border);
	}
};
