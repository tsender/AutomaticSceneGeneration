// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_msgs/ASGSrvsRunScenarioResponseConverter.h"

UASGSrvsRunScenarioResponseConverter::UASGSrvsRunScenarioResponseConverter() 
{
    _ServiceType = "auto_scene_gen_msgs/RunScenario";
}

bool UASGSrvsRunScenarioResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) 
{
	*Response = MakeShareable(new ROSMessages::auto_scene_gen_msgs::RunScenarioResponse);
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::RunScenarioResponse>(*Response);

	CastResponse->_Result = res.result_;
    return _bson_extract_child_response(res.full_msg_bson_, "values", CastResponse.Get());
}

bool UASGSrvsRunScenarioResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) 
{
    auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::RunScenarioResponse>(Response);
	res.result_ = CastResponse->_Result;
    _bson_append_response(res.values_bson_, CastResponse.Get());
	return true;
}

TSharedPtr<FROSBaseServiceResponse> UASGSrvsRunScenarioResponseConverter::AllocateConcreteResponse() 
{
    return MakeShareable(new ROSMessages::auto_scene_gen_msgs::RunScenarioResponse);
}
