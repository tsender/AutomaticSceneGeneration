// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/std_msgs/StdMsgsColorRGBAConverter.h"
#include "auto_scene_gen_msgs/msg/SceneCaptureSettings.h"
#include "ASGMsgsSceneCaptureSettingsConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsSceneCaptureSettingsConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsSceneCaptureSettingsConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::SceneCaptureSettings *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		msg->image_size = GetDoubleFromBSON(key + ".image_size", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		msg->draw_annotations = GetBoolFromBSON(key + ".draw_annotations", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->goal_sphere_thickness = GetDoubleFromBSON(key + ".goal_sphere_thickness", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		if (!UStdMsgsColorRGBAConverter::_bson_extract_child_color_rgba(b, key + ".goal_sphere_color", &msg->goal_sphere_color, LogOnErrors)) return false;

		msg->ortho_aerial = GetBoolFromBSON(key + ".ortho_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->perspective_aerial = GetBoolFromBSON(key + ".perspective_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->aerial_padding = (TArray<uint32>)GetInt32TArrayFromBSON(key + ".aerial_padding", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		
		msg->front_aerial = GetBoolFromBSON(key + ".front_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->front_left_aerial = GetBoolFromBSON(key + ".front_left_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->left_aerial = GetBoolFromBSON(key + ".left_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->back_left_aerial = GetBoolFromBSON(key + ".back_left_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->back_aerial = GetBoolFromBSON(key + ".back_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->back_right_aerial = GetBoolFromBSON(key + ".back_right_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->right_aerial = GetBoolFromBSON(key + ".right_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		msg->front_right_aerial = GetBoolFromBSON(key + ".front_right_aerial", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		msg->vehicle_start_pov = GetBoolFromBSON(key + ".vehicle_start_pov", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::SceneCaptureSettings *msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::SceneCaptureSettings *msg)
	{
		BSON_APPEND_DOUBLE(b, "image_size", msg->image_size);

		BSON_APPEND_BOOL(b, "draw_annotations", msg->draw_annotations);
		BSON_APPEND_DOUBLE(b, "goal_sphere_thickness", msg->goal_sphere_thickness);
		UStdMsgsColorRGBAConverter::_bson_append_child_color_rgba(b, "goal_sphere_color", &msg->goal_sphere_color);

		BSON_APPEND_BOOL(b, "ortho_aerial", msg->ortho_aerial);
		BSON_APPEND_BOOL(b, "perspective_aerial", msg->perspective_aerial);
		_bson_append_uint32_tarray(b, "aerial_padding", msg->aerial_padding);

		BSON_APPEND_BOOL(b, "front_aerial", msg->front_aerial);
		BSON_APPEND_BOOL(b, "front_left_aerial", msg->front_left_aerial);
		BSON_APPEND_BOOL(b, "left_aerial", msg->left_aerial);
		BSON_APPEND_BOOL(b, "back_left_aerial", msg->back_left_aerial);
		BSON_APPEND_BOOL(b, "back_aerial", msg->back_aerial);
		BSON_APPEND_BOOL(b, "back_right_aerial", msg->back_right_aerial);
		BSON_APPEND_BOOL(b, "right_aerial", msg->right_aerial);
		BSON_APPEND_BOOL(b, "front_right_aerial", msg->front_right_aerial);

		BSON_APPEND_BOOL(b, "vehicle_start_pov", msg->vehicle_start_pov);
	}
};
