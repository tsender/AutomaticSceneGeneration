// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_msgs/ASGSrvsWorkerIssueNotificationRequestConverter.h"


UASGSrvsWorkerIssueNotificationRequestConverter::UASGSrvsWorkerIssueNotificationRequestConverter() 
{
    _ServiceType = "auto_scene_gen_msgs/WorkerIssueNotification";
}

bool UASGSrvsWorkerIssueNotificationRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) 
{
    auto CastRequest = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::WorkerIssueNotificationRequest>(Request);
	return _bson_extract_child_request(req.full_msg_bson_, "args", CastRequest.Get());
}

bool UASGSrvsWorkerIssueNotificationRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) 
{
    auto CastRequest = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::WorkerIssueNotificationRequest>(Request);
    *BSONRequest = bson_new();
	_bson_append_request(*BSONRequest, CastRequest.Get());
    return true;
}

TSharedPtr<FROSBaseServiceRequest> UASGSrvsWorkerIssueNotificationRequestConverter::AllocateConcreteRequest() 
{
    return MakeShareable(new ROSMessages::auto_scene_gen_msgs::WorkerIssueNotificationRequest);
}
