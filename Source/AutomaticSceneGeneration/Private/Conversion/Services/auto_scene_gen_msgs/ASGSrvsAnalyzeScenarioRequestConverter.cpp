// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_msgs/ASGSrvsAnalyzeScenarioRequestConverter.h"


UASGSrvsAnalyzeScenarioRequestConverter::UASGSrvsAnalyzeScenarioRequestConverter() 
{
    _ServiceType = "auto_scene_gen_msgs/AnalyzeScenario";
}

bool UASGSrvsAnalyzeScenarioRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) 
{
    auto CastRequest = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::AnalyzeScenarioRequest>(Request);
	return _bson_extract_child_request(req.full_msg_bson_, "args", CastRequest.Get());
}

bool UASGSrvsAnalyzeScenarioRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) 
{
    auto CastRequest = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::AnalyzeScenarioRequest>(Request);
    *BSONRequest = bson_new();
	_bson_append_request(*BSONRequest, CastRequest.Get());
    return true;
}

TSharedPtr<FROSBaseServiceRequest> UASGSrvsAnalyzeScenarioRequestConverter::AllocateConcreteRequest() 
{
    return MakeShareable(new ROSMessages::auto_scene_gen_msgs::AnalyzeScenarioRequest);
}
