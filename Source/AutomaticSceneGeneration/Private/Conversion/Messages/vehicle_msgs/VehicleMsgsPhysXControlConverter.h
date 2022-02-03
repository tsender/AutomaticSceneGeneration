// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "ROSIntegration/Private/Conversion/Messages/BaseMessageConverter.h"
#include "vehicle_msgs/msg/PhysXControl.h"
#include "VehicleMsgsPhysXControlConverter.generated.h"

UCLASS()
class AUTOMATICSCENEGENERATION_API UVehicleMsgsPhysXControlConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UVehicleMsgsPhysXControlConverter();

	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);


	static bool _bson_extract_child_physx_control(bson_t *b, FString key, ROSMessages::vehicle_msgs::PhysXControl *p, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		p->longitudinal_velocity = GetDoubleFromBSON(key + ".longitudinal_velocity", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		p->steering_angle = GetDoubleFromBSON(key + ".steering_angle", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		p->handbrake = GetBoolFromBSON(key + ".handbrake", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		return true;
	}

	static void _bson_append_child_physx_control(bson_t *b, const char *key, const ROSMessages::vehicle_msgs::PhysXControl *pc)
	{
		bson_t vec;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &vec);
		_bson_append_physx_control(&vec, pc);
		bson_append_document_end(b, &vec);
	}

	static void _bson_append_physx_control(bson_t *b, const ROSMessages::vehicle_msgs::PhysXControl *pc)
	{
		BSON_APPEND_DOUBLE(b, "longitudinal_velocity", pc->longitudinal_velocity);
		BSON_APPEND_DOUBLE(b, "steering_angle", pc->steering_angle);
		BSON_APPEND_BOOL(b, "handbrake", pc->handbrake);
	}
};
