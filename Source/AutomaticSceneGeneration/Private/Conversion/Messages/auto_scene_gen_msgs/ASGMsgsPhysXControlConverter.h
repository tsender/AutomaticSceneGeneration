// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "auto_scene_gen_msgs/msg/PhysXControl.h"
#include "ASGMsgsPhysXControlConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsPhysXControlConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsPhysXControlConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::PhysXControl *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->longitudinal_velocity = GetDoubleFromBSON(key + ".longitudinal_velocity", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->steering_angle = GetDoubleFromBSON(key + ".steering_angle", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->handbrake = GetBoolFromBSON(key + ".handbrake", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::PhysXControl *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::PhysXControl *msg)
	{
		BSON_APPEND_DOUBLE(b, "longitudinal_velocity", msg->longitudinal_velocity);
		BSON_APPEND_DOUBLE(b, "steering_angle", msg->steering_angle);
		BSON_APPEND_BOOL(b, "handbrake", msg->handbrake);
	}
};
