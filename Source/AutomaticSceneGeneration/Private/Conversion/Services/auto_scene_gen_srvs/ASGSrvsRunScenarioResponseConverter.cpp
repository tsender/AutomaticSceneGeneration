// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_srvs/ASGSrvsRunScenarioResponseConverter.h"

UASGSrvsRunScenarioResponseConverter::UASGSrvsRunScenarioResponseConverter() 
{
    _ServiceType = "auto_scene_gen_srvs/RunScenario";
}

bool UASGSrvsRunScenarioResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) 
{
	*Response = MakeShareable(new ROSMessages::auto_scene_gen_srvs::FRunScenarioResponse);
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_srvs::FRunScenarioResponse>(*Response);

	CastResponse->_Result = res.result_;
    return _bson_extract_child_response(res.full_msg_bson_, "values", CastResponse.Get());
}

bool UASGSrvsRunScenarioResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) 
{
    auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_srvs::FRunScenarioResponse>(Response);

	res.result_ = CastResponse->_Result;
    _bson_append_response(res.values_bson_, CastResponse.Get());
	return true;
}

TSharedPtr<FROSBaseServiceResponse> UASGSrvsRunScenarioResponseConverter::AllocateConcreteResponse() 
{
    return MakeShareable(new ROSMessages::auto_scene_gen_srvs::FRunScenarioResponse);
}
