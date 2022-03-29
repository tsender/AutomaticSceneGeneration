#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/std_msgs/StdMsgsHeaderConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/geometry_msgs/GeometryMsgsPoseConverter.h"
#include "ROSIntegration/Private/Conversion/Messages/geometry_msgs/GeometryMsgsTwistConverter.h"
#include "auto_scene_gen_msgs/msg/OdometryWithoutCovariance.h"

#include "ASGMsgsOdometryWithoutCovarianceConverter.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API UASGMsgsOdometryWithoutCovarianceConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UASGMsgsOdometryWithoutCovarianceConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_msg(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance * msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;
		if (!UStdMsgsHeaderConverter::_bson_extract_child_header(b, key + ".header", &msg->header)) return false;
		msg->child_frame_id = GetFStringFromBSON(key + ".child_frame_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		if (!UGeometryMsgsPoseConverter::_bson_extract_child_pose(b, key + ".pose", &msg->pose)) return false;
		if (!UGeometryMsgsTwistConverter::_bson_extract_child_twist(b, key + ".twist", &msg->twist)) return false;
		return true;
	}

	static void _bson_append_child_msg(bson_t *b, const char *key, const ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance * msg)
	{
		bson_t child;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &child);
		_bson_append_msg(&child, msg);
		bson_append_document_end(b, &child);
	}

	static void _bson_append_msg(bson_t *b, const ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance * msg)
	{
		UStdMsgsHeaderConverter::_bson_append_child_header(b, "header", &msg->header);
		BSON_APPEND_UTF8(b, "child_frame_id", TCHAR_TO_UTF8(*msg->child_frame_id));
		UGeometryMsgsPoseConverter::_bson_append_child_pose(b, "pose", &msg->pose);
		UGeometryMsgsTwistConverter::_bson_append_child_twist(b, "twist", &msg->twist);
	}
};
