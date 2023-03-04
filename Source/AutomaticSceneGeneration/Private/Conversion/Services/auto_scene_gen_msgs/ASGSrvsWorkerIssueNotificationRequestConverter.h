// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Conversion/Services/BaseRequestConverter.h"
#include "auto_scene_gen_msgs/srv/WorkerIssueNotificationRequest.h"
#include "ASGSrvsWorkerIssueNotificationRequestConverter.generated.h"


UCLASS()
class AUTOMATICSCENEGENERATION_API UASGSrvsWorkerIssueNotificationRequestConverter : public UBaseRequestConverter
{
	GENERATED_BODY()

public:
	UASGSrvsWorkerIssueNotificationRequestConverter();

	virtual bool ConvertIncomingRequest(ROSBridgeCallServiceMsg &req, TSharedPtr<FROSBaseServiceRequest> Request) override;
	virtual bool ConvertOutgoingRequest(TSharedPtr<FROSBaseServiceRequest> Request, bson_t** BSONRequest) override;

	virtual TSharedPtr<FROSBaseServiceRequest> AllocateConcreteRequest() override;

	static bool _bson_extract_child_request(bson_t *b, FString key, ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest *request, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		request->worker_id = UBaseMessageConverter::GetInt32FromBSON(key + ".worker_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->issue_id = UBaseMessageConverter::GetInt32FromBSON(key + ".issue_id", b, KeyFound, LogOnErrors); if (!KeyFound) return false;
		request->message = UBaseMessageConverter::GetFStringFromBSON(key + ".message", b, KeyFound, LogOnErrors); if (!KeyFound) return false;

		return true;
	}

	static void _bson_append_request(bson_t *b, const ROSMessages::auto_scene_gen_msgs::FWorkerIssueNotificationRequest *request)
	{
		BSON_APPEND_INT32(b, "worker_id", request->worker_id);
		BSON_APPEND_INT32(b, "issue_id", request->issue_id);
		BSON_APPEND_UTF8(b, "message", TCHAR_TO_UTF8(*request->message));
	}
};
