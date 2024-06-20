#pragma once

#include "CoreMinimal.h"
#include "Conversion/Messages/BaseMessageConverter.h"
#include "std_msgs/UInt8MultiArray.h"
#include "Conversion/Messages/std_msgs/StdMsgsMultiArrayLayoutConverter.h"
#include "StdMsgsUInt8MultiArrayConverter.generated.h"


UCLASS()
class ROSINTEGRATION_API UStdMsgsUInt8MultiArrayConverter : public UBaseMessageConverter
{
	GENERATED_BODY()

public:
	UStdMsgsUInt8MultiArrayConverter();
	virtual bool ConvertIncomingMessage(const ROSBridgePublishMsg* message, TSharedPtr<FROSBaseMsg> &BaseMsg);
	virtual bool ConvertOutgoingMessage(TSharedPtr<FROSBaseMsg> BaseMsg, bson_t** message);

	static bool _bson_extract_child_uint8_multi_array(bson_t *b, FString key, ROSMessages::std_msgs::UInt8MultiArray *msg, bool LogOnErrors = true)
	{
		bool KeyFound = false;

		KeyFound = UStdMsgsMultiArrayLayoutConverter::_bson_extract_child_multi_array_layout(b, key + ".layout", &msg->layout, LogOnErrors); if (!KeyFound) return false;
		msg->data = rosbridge2cpp::Helper::get_binary_by_key(TCHAR_TO_UTF8(*(key + ".data")), *b, msg->layout.dim[0].size, KeyFound);

		return true;
	}

	static void _bson_append_child_uint8_multi_array(bson_t *b, const char *key, const ROSMessages::std_msgs::UInt8MultiArray *fma)
	{
		bson_t layout;
		BSON_APPEND_DOCUMENT_BEGIN(b, key, &layout);
		_bson_append_uint8_multi_array(&layout, fma);
		bson_append_document_end(b, &layout);
	}

	static void _bson_append_uint8_multi_array(bson_t *b, const ROSMessages::std_msgs::UInt8MultiArray *msg)
	{
		UStdMsgsMultiArrayLayoutConverter::_bson_append_child_multi_array_layout(b, "layout", &msg->layout);
		bson_append_binary(b, "data", -1, BSON_SUBTYPE_BINARY, msg->data, msg->layout.dim[0].size);
	}
};