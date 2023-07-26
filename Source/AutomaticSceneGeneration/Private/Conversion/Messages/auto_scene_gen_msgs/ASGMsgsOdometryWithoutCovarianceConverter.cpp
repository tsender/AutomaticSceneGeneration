#include "Conversion/Messages/auto_scene_gen_msgs/ASGMsgsOdometryWithoutCovarianceConverter.h"


UASGMsgsOdometryWithoutCovarianceConverter::UASGMsgsOdometryWithoutCovarianceConverter()
{
	_MessageType = "auto_scene_gen_msgs/OdometryWithoutCovariance";
}

bool UASGMsgsOdometryWithoutCovarianceConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	auto msg = new ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance();
	BaseMsg = TSharedPtr<FROSBaseMsg>(msg);
	return _bson_extract_child_msg(message->full_msg_bson_, "msg", msg);
}

bool UASGMsgsOdometryWithoutCovarianceConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto CastMsg = StaticCastSharedPtr<ROSMessages::auto_scene_gen_msgs::OdometryWithoutCovariance>(BaseMsg);
	*message = bson_new();
	_bson_append_msg(*message, CastMsg.Get());
	return true;
}