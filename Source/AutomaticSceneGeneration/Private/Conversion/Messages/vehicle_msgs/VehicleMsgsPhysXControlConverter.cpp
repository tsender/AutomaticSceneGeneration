// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Messages/vehicle_msgs/VehicleMsgsPhysXControlConverter.h"


UVehicleMsgsPhysXControlConverter::UVehicleMsgsPhysXControlConverter() 
{
    _MessageType = "vehicle_msgs/PhysXControl"; 
}

bool UVehicleMsgsPhysXControlConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg) 
{
    auto p = new ROSMessages::vehicle_msgs::PhysXControl();
	BaseMsg = TSharedPtr<FROSBaseMsg>(p);
	return _bson_extract_child_physx_control(message->full_msg_bson_, "msg", p);
}

bool UVehicleMsgsPhysXControlConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message) 
{
    auto PhysXControl = StaticCastSharedPtr<ROSMessages::vehicle_msgs::PhysXControl>(BaseMsg);

	*message = bson_new();
	_bson_append_physx_control(*message, PhysXControl.Get());

	return true;
}
