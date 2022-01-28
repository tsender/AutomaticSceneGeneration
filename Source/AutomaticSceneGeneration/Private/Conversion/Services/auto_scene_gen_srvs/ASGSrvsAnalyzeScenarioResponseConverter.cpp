// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_srvs/ASGSrvsAnalyzeScenarioResponseConverter.h"

UASGSrvsAnalyzeScenarioResponseConverter::UASGSrvsAnalyzeScenarioResponseConverter() 
{
    _ServiceType = "auto_scene_gen_srvs/AnalyzeScenario";
}

bool UASGSrvsAnalyzeScenarioResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) 
{
	*Response = MakeShareable(new ROSMessages::auto_scene_gen_srvs::FAnalyzeScenarioResponse);
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_srvs::FAnalyzeScenarioResponse>(*Response);

	CastResponse->_Result = res.result_;
    return _bson_extract_child_response(res.full_msg_bson_, "values", CastResponse.Get());
}

bool UASGSrvsAnalyzeScenarioResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) 
{
    auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_srvs::FAnalyzeScenarioResponse>(Response);

	res.result_ = CastResponse->_Result;
    _bson_append_response(res.values_bson_, CastResponse.Get());
	return true;
}

TSharedPtr<FROSBaseServiceResponse> UASGSrvsAnalyzeScenarioResponseConverter::AllocateConcreteResponse() 
{
    return MakeShareable(new ROSMessages::auto_scene_gen_srvs::FAnalyzeScenarioResponse);
}
