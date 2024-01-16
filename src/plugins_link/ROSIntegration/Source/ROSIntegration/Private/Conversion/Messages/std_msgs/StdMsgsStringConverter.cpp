#include "Conversion/Messages/std_msgs/StdMsgsStringConverter.h"


UStdMsgsStringConverter::UStdMsgsStringConverter()
{
	_MessageType = "std_msgs/String";
}

bool UStdMsgsStringConverter::ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg)
{
	bool KeyFound = false;

	FString Data = GetFStringFromBSON(TEXT("msg.data"), message->full_msg_bson_, KeyFound);
	if (!KeyFound) return false;

	BaseMsg = TSharedPtr<FROSBaseMsg>(new ROSMessages::std_msgs::String(Data));
	return true;
}

bool UStdMsgsStringConverter::ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message)
{
	auto StringMessage = StaticCastSharedPtr<ROSMessages::std_msgs::String>(BaseMsg);

	bson_t bson;
	bson_init(&bson);
	BSON_APPEND_UTF8(&bson, "data", TCHAR_TO_UTF8(*StringMessage->_Data));

	*message = bson_copy(&bson);
	bson_destroy(&bson);

	char *json_str = bson_as_json(*message, NULL);
	bson_free(json_str);

	return true;
}