// Fill out your copyright notice in the Description page of Project Settings.


#include "Conversion/Services/auto_scene_gen_msgs/ASGSrvsSceneCaptureResponseConverter.h"

UASGSrvsSceneCaptureResponseConverter::UASGSrvsSceneCaptureResponseConverter() 
{
    _ServiceType = "auto_scene_gen_msgs/SceneCapture";
}

bool UASGSrvsSceneCaptureResponseConverter::ConvertIncomingResponse(const ROSBridgeServiceResponseMsg &res, TSharedRef<TSharedPtr<FROSBaseServiceResponse>> Response) 
{
	*Response = MakeShareable(new ROSMessages::auto_scene_gen_msgs::FSceneCaptureResponse);
	auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::FSceneCaptureResponse>(*Response);

	CastResponse->_Result = res.result_;
    return _bson_extract_child_response(res.full_msg_bson_, "values", CastResponse.Get());
}

bool UASGSrvsSceneCaptureResponseConverter::ConvertOutgoingResponse(TSharedPtr<FROSBaseServiceResponse> Response, ROSBridgeServiceResponseMsg &res) 
{
    auto CastResponse = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::FSceneCaptureResponse>(Response);
	res.result_ = CastResponse->_Result;
    _bson_append_response(res.values_bson_, CastResponse.Get());
	return true;
}

TSharedPtr<FROSBaseServiceResponse> UASGSrvsSceneCaptureResponseConverter::AllocateConcreteResponse() 
{
    return MakeShareable(new ROSMessages::auto_scene_gen_msgs::FSceneCaptureResponse);
}
