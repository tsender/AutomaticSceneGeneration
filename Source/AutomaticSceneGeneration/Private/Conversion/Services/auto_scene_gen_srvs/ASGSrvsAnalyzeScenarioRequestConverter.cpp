// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_srvs/ASGSrvsAnalyzeScenarioRequestConverter.h"


UASGSrvsAnalyzeScenarioRequestConverter::UASGSrvsAnalyzeScenarioRequestConverter() 
{
    _ServiceType = "auto_scene_gen_srvs/AnalyzeScenario";
}

bool UASGSrvsAnalyzeScenarioRequestConverter::ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) 
{
    auto CastRequest = StaticCastSharedPtr<auto_scene_gen_srvs::FAnalyzeScenarioRequest>(Request);
	return _bson_extract_child_request(req.full_msg_bson_, "args", CastRequest.Get());
}

bool UASGSrvsAnalyzeScenarioRequestConverter::ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) 
{
    auto CastRequest = StaticCastSharedPtr<auto_scene_gen_srvs::FAnalyzeScenarioRequest>(Request);
    *BSONRequest = bson_new();
	_bson_append_request(*BSONRequest, CastRequest.Get());
    return true;
}

TSharedPtr<FROSBaseServiceRequest> UASGSrvsAnalyzeScenarioRequestConverter::AllocateConcreteRequest() 
{
    return MakeShareable(new auto_scene_gen_srvs::FAnalyzeScenarioRequest);
}
