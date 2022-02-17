// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsEnableStatusConverter.h"


UASGMsgsEnableStatusConverter::UASGMsgsEnableStatusConverter() 
{
    _MessageType = "auto_scene_gen_msgs/EnableStatus";
}

bool UASGMsgsEnableStatusConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
    auto msg = new ROSMessages::auto_scene_gen_msgs::EnableStatus();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_msg(message->full_msg_bson_, "msg", msg);
}

bool UASGMsgsEnableStatusConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
    auto CastMsg = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::EnableStatus>(BaseMsg);

	*message = bson_new();
	_bson_append_msg(*message, CastMsg.Get());

	return true;
}
