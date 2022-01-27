// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsStructuralSceneActorInfoConverter.h"


UASGMsgsStructuralSceneActorInfoConverter::UASGMsgsStructuralSceneActorInfoConverter() 
{
    _MessageType = "auto_scene_gen_msgs/StructuralSceneActorInfo";
}

bool UASGMsgsStructuralSceneActorInfoConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
    auto msg = new ROSMessages::auto_scene_gen_msgs::StructuralSceneActorInfo();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_msg(message->full_msg_bson_, "msg", msg);
}

bool UASGMsgsStructuralSceneActorInfoConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
    auto CastMsg = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::StructuralSceneActorInfo>(BaseMsg);

	*message = bson_new();
	_bson_append_msg(*message, CastMsg.Get());

	return true;
}
