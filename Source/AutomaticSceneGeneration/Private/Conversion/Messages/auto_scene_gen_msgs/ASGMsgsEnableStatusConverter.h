// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "auto_scene_gen_msgs/msg/EnableStatus.h"
#include "ASGMsgsEnableStatusConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsEnableStatusConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsEnableStatusConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::EnableStatus *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->enabled = GetBoolFromBSON(key + ".enabled", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::EnableStatus *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::EnableStatus *msg)
	{
		BSON_APPEND_BOOL(b, "enabled", msg->enabled);
	}
};
